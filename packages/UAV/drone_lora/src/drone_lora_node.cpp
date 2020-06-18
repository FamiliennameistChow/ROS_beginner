/**
 * @file    drone_lora_node.cpp
 * @author  Kumaron
 * @date    2019.05.05
 * @brief   drone lora node
 *          
 *
 */
/**
 ***********通讯协议说明*********** 
 * 
 * Lora定点发送 
 *      地址高位 地址低位 通道 
 * 帧头 
 *      0xfe 0xfe
 * 发送方
 *      node.seq
 * 接收方
 *      node.seq
 * 消息类型
 *      查询传感器数据 0xaa
 *      查询电量数据
 * 消息内容
 *      传感器数据
 *          N-传感器数量， N*3字节传感器数据
 *      电量数据
 *          3字节电量数据
 * CRC
 *      由发送方至消息内容各位取异或
 * 帧尾
 *      0xfd 0xfd
 ********************************
*/

#include <ros/ros.h>
#include <serial/serial.h>  //需要安装ROS官方的serial包,并且在Cmakelist文件中的find_package内加入serial
#include <iostream>
#include <inttypes.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <errno.h>
#include <vector>
#include <string>

#include <mavros_msgs/GlobalPositionTarget.h>   // 获取GPS信息

using namespace std;
//测试用
#define DEBUG 1

/**
 * 类型定义
*/
typedef struct WaterNode    //水质监测节点相关
{
    uint8_t lora_addr_high;
    uint8_t lora_addr_low;
    uint8_t lora_channel;
    uint8_t data[20];   //保存节点传感器数据
    uint8_t seq;        //节点编号
    bool is_check_success = false;
}water_node_t;

typedef struct LoraMsg      //lora通信相关
{
    uint8_t head = 0xfe;    //帧头
    uint8_t sender = 0x00;  //发送方默认无人机为0x00
    uint8_t receiver;       //接收方
    uint8_t msg_type;       //消息类型
    uint8_t msg_cont;       //消息内容
    uint8_t crc;            //校验
    uint8_t tail = 0xfd;    //帧尾
    uint8_t msg_type_query = 0xaa; //消息类型——查询
    uint8_t msg_cont_query = 0xaa; //消息内容——查询
    uint8_t msg_type_sensor_data = 0x03; //消息类型——传感器数据
    uint8_t msg_type_power_data  = 0x02; //消息类型——电量数据
    int max_received_bytes = 256; //一次最多接收字节数
}lora_msg_t;


/**
 * 配置socket
*/
int socket_config(  int         client_sockfd,
                    const       string &address,
                    uint16_t    port,
                    uint8_t     *socket_msg
                )
{
    int addrlen;
    struct sockaddr_in ser_addr, cli_addr;
    // char msg[MAX_MSG_SIZE]; //缓冲区
    client_sockfd = socket(AF_INET, SOCK_STREAM, 0);//创建连接的socket
    if(client_sockfd<0)//连接失败
    {
        #ifdef DEBUG
        fprintf(stderr, "Socket Error: %s\n", strerror(errno));
        #endif
        exit(1);
    }
    #ifdef DEBUG
    printf("Create socket success\n");
    #endif
    // 初始化客户端地址
    addrlen = sizeof(struct sockaddr_in);
    bzero(&ser_addr, addrlen);
    cli_addr.sin_family = AF_INET;
    cli_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    cli_addr.sin_port = 0;
    if(bind(client_sockfd, (struct sockaddr*)&cli_addr, addrlen)<0)
    {
        // 绑定失败
        #ifdef DEBUG
        fprintf(stderr, "Bind Error: %s\n", strerror(errno));
        #endif
        exit(1);
    }
    #ifdef DEBUG
    printf("Bind success\n");
    #endif
    // 初始化服务器地址
    addrlen = sizeof(struct sockaddr_in);
    bzero(&ser_addr, addrlen);
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = inet_addr("218.4.207.179");
    ser_addr.sin_port = htons(12200);
    int connect_try_cnt = 0;
    while(1)
    {
        if(connect(client_sockfd, (struct sockaddr*)&ser_addr, addrlen)!=0)// 请求连接
        {
            // 连接失败
            #ifdef DEBUG
            fprintf(stderr, "Connect Error: %s\n", strerror(errno));
            #endif
            // close(client_sockfd);
            // exit(1);
            connect_try_cnt++;
            if(connect_try_cnt>99)
            {
                #ifdef DEBUG
                cout << "Too many trys failed." << endl;
                #endif
                exit(1);
            }
            #ifdef DEBUG
            cout << "Reconnecting ..." << endl;
            #endif
            continue;
        }
        else
        {
            #ifdef DEBUG
            cout << "Connect success" << endl;
            #endif
            break;
        }
    }
}

/**
 * 配置串口
*/
int serial_config(  serial::Serial      *serial,
                    const string        &port, 
                    const uint32_t      baudrate=9600
                )
{
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    //UART0 ttyS0|UART1 ttyTHS2|UART2 ttyTHS1
    serial->setPort(port);
    serial->setBaudrate(baudrate);
    serial->setStopbits(serial::stopbits_one);
    serial->setParity(serial::parity_none);
    serial->setTimeout(timeout);
    try
    {
        serial->open();
    }
    catch(serial::IOException& e)
    {
        #ifdef DEBUG
        ROS_ERROR_STREAM("Unable to open serial port!");
        #endif
        return -1; 
    }
}

/**
 * 配置节点
*/
void node_config(   vector<water_node_t> * const water_nodes,
                    uint8_t lora_addr_high,
                    uint8_t lora_addr_low,
                    uint8_t lora_channel
                )
{
    water_node_t node;
    node.lora_addr_high     = lora_addr_high;
    node.lora_addr_low      = lora_addr_low;
    node.lora_channel       = lora_channel;
    water_nodes->push_back(node);
}

/**
 * 节点抄表
 * !!!CRITIC!!!
 * 1. 如果节点lora失效，如何放弃该节点去查询下一节点（使用GPS？）
 * 2. 如果变更河道，如何获取该河道的节点的地址信道（使用socket由服务器给出？、想办法使得节点能够接收命令并返回自身地址信道）
*/
void read_node( water_node_t        *node, 
                serial::Serial      *serial, 
                const lora_msg_t    lora_msg,
                uint8_t             *send_buffer,
                uint8_t             *read_buffer,
                uint8_t             *send_to_server,
                const int           client_socket_fd
            )
{
    #ifdef DEBUG
    cout << "Querying node ..." << (int)node->seq << endl;
    #endif
    uint8_t sensor_num_received;
    while(ros::ok())
    {
        //定点发送抄表命令至节点
        send_buffer[0]  = node->lora_addr_high;     //定点发送 
        send_buffer[1]  = node->lora_addr_low;
        send_buffer[2]  = node->lora_channel;
        send_buffer[3]  = lora_msg.head;            //帧头
        send_buffer[4]  = lora_msg.head;
        send_buffer[5]  = lora_msg.sender;          //发送方
        send_buffer[6]  = lora_msg.receiver;        //接收方
        send_buffer[7]  = lora_msg.msg_type;        //消息类型
        send_buffer[8]  = lora_msg.msg_cont_query;  //抄表
        send_buffer[9]  =   send_buffer[5]^
                            send_buffer[6]^
                            send_buffer[7]^
                            send_buffer[8];         //校验
        send_buffer[10] = lora_msg.tail;            //帧尾
        send_buffer[11] = lora_msg.tail;
        serial->write(send_buffer, 12); //定点发送抄表命令
        sleep(2);
        //接收返回消息
        size_t number_of_data_received = serial->available();
        if(number_of_data_received < lora_msg.max_received_bytes)//接收字节数在允许范围内 
        {
            number_of_data_received = serial->read(read_buffer, serial->available());
            for(int i = 0; i < number_of_data_received; i ++)
            {
                node->data[i] = read_buffer[i];
                #ifdef DEBUG //测试时输出接收到十六进制数据
                cout << hex << (node->data[i] & 0xff) << endl;
                #endif
            }
            uint8_t crc_calculated = 0x00;
            uint8_t crc_received = 0x00;
            if( read_buffer[0] == lora_msg.head &&
                read_buffer[1] == lora_msg.head &&
                read_buffer[2] == node->seq &&
                read_buffer[3] == 0x00)
            {
                crc_calculated = read_buffer[2]^read_buffer[3];
                //传感器数据
                if(read_buffer[4] == lora_msg.msg_type_sensor_data)
                {   //根据传感器数量N进行校验
                    int sensor_num = (int)read_buffer[5];
                    sensor_num_received = read_buffer[5];
                    for(int i=0; i<sensor_num*3; i++)
                    {
                        crc_calculated ^= read_buffer[6+i];
                    }
                    crc_received = read_buffer[6+sensor_num*3];
                    #ifdef DEBUG
                    cout << "crc calculated: "  << crc_calculated << endl;
                    cout << "crc received:"     << crc_received << endl;
                    #endif
                    if( crc_calculated == crc_received &&
                        read_buffer[6+sensor_num*3+1] == lora_msg.tail &&
                        read_buffer[6+sensor_num*3+1] == lora_msg.tail)
                    {
                        #ifdef DEBUG
                        cout << "Query node " << (int)node->seq << " succeed!"<< endl;
                        #endif                
                        node->is_check_success = true;  
                        break;  
                    }
                    else
                    {
                        #ifdef DEBUG
                        cout << "Query node " << (int)node->seq << " failed!"<< endl;
                        #endif                
                        node->is_check_success = false;
                        continue;    
                    }
                }
                //电量数据
                if(read_buffer[4] == lora_msg.msg_type_power_data)  //电量数据
                {
                    crc_calculated ^=   read_buffer[4]^read_buffer[5]^
                                        read_buffer[6]^read_buffer[7];    
                    crc_received = read_buffer[8];
                    #ifdef DEBUG
                    cout << "crc calculated: "  << crc_calculated << endl;
                    cout << "crc received:"     << crc_received << endl;
                    #endif         
                    if( crc_calculated == crc_received&&
                        read_buffer[9]  == lora_msg.tail &&
                        read_buffer[10] == lora_msg.tail)
                    {
                        #ifdef DEBUG
                        cout << "Queried node " << (int)node->seq << " with only power data, check succeed"<< endl;
                        #endif                
                        node->is_check_success = true;  
                        break;  
                    }
                    else
                    {
                        #ifdef DEBUG
                        cout << "Queried node " << (int)node->seq << " with only power data, check failed"<< endl;
                        #endif                
                        node->is_check_success = false;
                        continue;
                    }
                }
            }
        }//这里不写else是避免重复发送太多次抄表命令
        else
        {
            #ifdef DEBUG
            ROS_WARN("Received more than %d bytes", lora_msg.max_received_bytes);
            #endif
            sleep(1);
            continue;
        }
    }
    #ifdef DEBUG
    cout << "Now sending data to server using socket" << endl;
    #endif
    //TODO:将接收到的数据存入send_to_server
    send_to_server[0]   = 0xfe;                 //帧头
    send_to_server[1]   = 0xfe;                 
    send_to_server[2]   = 0x00;                 //发送方——无人机
    send_to_server[3]   = 0xff;                 //接受方——服务器
    send_to_server[4]   = node->seq;            //节点序列号
    send_to_server[5]   = sensor_num_received;  //传感器数目
    /*
    send_to_server[6]   = ;                       //温度
    send_to_server[7]   = ;
    send_to_server[8]   = ;
    send_to_server[9]   = ;                       //pH值
    send_to_server[10]  = ;
    send_to_server[11]  = ;
    send_to_server[12]  = ;                       //浊度
    send_to_server[13]  = ;
    send_to_server[14]  = ;
    send_to_server[15]  = ;                       //电导率
    send_to_server[16]  = ;
    send_to_server[17]  = ;
    */
    send_to_server[18] = 0xfd;                  //帧尾
    send_to_server[19] = 0xfd;
    //socket发送至服务器
    write(client_socket_fd, send_to_server, sizeof(send_to_server));
    #ifdef DEBUG
    cout << "Date sent to server" << endl;
    #endif

}

/**
 * main
*/
int main(int argc, char *argv[])
{
    //ROS初始化
    ros::init(argc, argv, "drone_lora");
    ros::NodeHandle nh;

    //配置TCP/IP的socket连接
    int client_sockfd;
    string socket_address   = "218.4.207.179";  //IP地址
    uint16_t socket_port    = 12200;            //IP端口
    const int msg_size      = 256;              //缓存区大小
    uint8_t socket_msg[msg_size];               //socket缓存区
    socket_config(client_sockfd, socket_address, socket_port, socket_msg);
    
    //配置串口
    serial::Serial lora;
    uint32_t lora_baud  = 115200;           //串口波特率
    string lora_port    = "/dev/ttyUSB0";   //串口设备号
    serial_config(&lora, lora_port, lora_baud);

    //配置节点
    vector<water_node_t> water_nodes = vector<water_node_t>();
    /**
     * 测试
     * 设定无人机为 0x00 0x00 0x00 115200
     * 设定节点1为  0x01 0x01 0x01 115200
    */
    // 依次为addr_high, addr_low, channel
    node_config(&water_nodes, 0x01, 0x01, 0x01); //节点1
    // node_config(&water_nodes, 0x02, 0x02, 0x02); //节点2
    // node_config(&water_nodes, 0x03, 0x03, 0x03); //节点3
    #ifdef DEBUG
    int cnt = 0;
    for(vector<water_node_t>::iterator it = water_nodes.begin();
    it<water_nodes.end(); it++)
    {
        cout << "________" << "node " << cnt << "________" << endl;
        cout << "addr_high: " << it->lora_addr_high << endl;
        cout << "addr_low: " << it->lora_addr_low << endl;
        cout << "channel: " << it->lora_channel << endl;
        cout << endl;
        cnt++;
    }
    #endif

    //配置串口命令
    lora_msg_t read_cmd;
    read_cmd.msg_type = read_cmd.msg_type_query;
    read_cmd.msg_cont = read_cmd.msg_cont_query;
    //申请动态分配内存
    uint8_t *send_to_node_buffer        = new uint8_t[20];  
    uint8_t *receive_from_node_buffer   = new uint8_t[20];
    uint8_t *send_to_server_buffer      = new uint8_t[20];     
    ros::Rate rate(20.0);
    //对节点进行抄表，并将数据上传至服务器
    for(vector<water_node_t>::iterator it = water_nodes.begin();
        it<water_nodes.end(); it++)
    {
        read_node(  &(*it),             //节点
                    &lora,              //串口
                    read_cmd,           //命令
                    send_to_node_buffer,        //发送给节点的数据
                    receive_from_node_buffer,   //从节点接收的数据
                    send_to_server_buffer,      //发送给服务器的数据
                    client_sockfd       //socket
                );
    }

    #ifdef DEBUG
    cout << "Deleting [] ..." << endl;
    #endif
    //释放动态分配的内存
    delete[] send_to_node_buffer;
    delete[] receive_from_node_buffer;
    delete[] send_to_server_buffer;

    return EXIT_SUCCESS;
}
