 /********************************************************
 * PathFinder.hpp
 * 3D rrt use octomap method
 * 
 * Author： Born Chow
 * Date: 2020.08.28
 * 
 * 说明: 在octomap上实现rrt导航,用于小车的起伏地形导航,
 * 这里实现前端rrt 路径查找 RRTPathFinder类
 * 
 * How to use:
 *  
 *  1. 申明
 * 	RRTPathFinder *path_finder = new RRTPathFinder();
 * 
 *  2. 初始化变量
 *  path_finder->initParam(search_radius_, th_stdev_, setp_eta_, max_iter_, car_body_size_, n_, true);path_finder->updateMap(map_tree_);
 * 　
 *  3. 路径查找
 *  path_finder->updateMap(map_tree_);
	path_finder->pathSearch(base_start, base_goal);
	if(path_finder->getPath(rrt_path_)){
		visRRTPath(rrt_path_);
	}
 * 
 ******************************************************/
// #include <Eigen/Dense>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>

#include "data_type.h"
#include <unordered_map>
#include "tictoc.h"

// #define DEBUG true;

struct rrtNode{
    octomap::point3d father_node;
    float cost;
};


// typedef octomap::point3d Point3D;
typedef std::unordered_map<octomap::point3d, rrtNode> Point_Pair_Set; //储存(当前节，(父节点, 当前节点代价))对
typedef std::unordered_map<octomap::point3d, rrtNode>::iterator TreeIterator;

class RRTPathFinder
{
private:

    octomap::point3d start_pt_;
    octomap::point3d end_pt_;
    octomap::point3d new_pt_, rand_pt_, nearest_pt_;
    Point_Pair_Set rrt_tree_set_;  // 八叉树无序容器， 保存【当前节，(父节点, 当前节点代价)】对
    rrtNode this_node_;

	vector<octomap::point3d> rrt_path_; //rrt最终路径节点集合
	vector<octomap::point3d> vis_new_pt_set_; //rrt采样点集合
	vector<octomap::point3d> vis_block_pt_set_; //rrt阻断的节点集合
	vector<pair<octomap::point3d, octomap::point3d> > vis_rrt_tree_set_; // rrt树

    // rrt 相关参数
    int rrt_count_; // 迭代次数 
    int rrt_count_th_; //最大迭代次数
    float eta_; // 步进长度
    float search_r_; // rewire阶段搜索半径
	bool search_Finished_; // 是否完成搜索
	
	// 地形分析相关
	float th_stdev_; // 地形平整度阈值

    // 地图相关
    shared_ptr<octomap::OcTree> map_tree_;
    float map_resolution_;
    // 模型尺寸，用于膨胀地图;
    octomap::point3d carBodySize;

	// ros::NodeHandle n_;
	// ros::Publisher rrt_tree_pub_;
	// ros::Publisher rrt_path_pub_;
	// ros::Publisher vis_pub_;
	// bool is_vis_;

private:
    // rrt 流程
    octomap::point3d Near(Point_Pair_Set &V , octomap::point3d p);
	octomap::point3d Steer(octomap::point3d nearest_point, octomap::point3d rand_point, float eta_);
    int collisionFree(octomap::point3d nearest_point, octomap::point3d new_point);
    void rewirteTree(Point_Pair_Set &V, octomap::point3d new_point, octomap::point3d nearest_point, float search_r);
	bool isGoal(octomap::point3d new_point, octomap::point3d end_point);
	void backTree(octomap::point3d p, octomap::point3d p_init, vector<octomap::point3d> &rrt_tree);


private:
    double getRandData(int min, int max)
	{
		double m1=(double)(rand()%101)/101; // 计算 0，1之间的随机小数,得到的值域近似为(0,1)
		min++;  //将 区间变为(min+1,max),
		double m2=(double)((rand()%(max-min+1))+min); //计算 min+1,max 之间的随机整数，得到的值域为[min+1,max]
		m2=m2-1; //令值域为[min,max-1]
		return m1+m2;  //返回值域为(min,max),为所求随机浮点数
	}

    float Norm(octomap::point3d p1, octomap::point3d p2){
        //　返回两点的欧式距离(x,y方向)
	    return pow( pow(p1(0)-p2(0), 2) + pow(p1(1)-p2(1),2), 0.5);
    }

	bool point_equal(octomap::point3d p, octomap::point3d p1){
		float half_szie = map_resolution_ / 2; 
		if(abs(p(0) - p1(0)) <= half_szie && abs(p(1) - p1(1)) <= half_szie && abs(p(2) - p1(2)) <= half_szie){
			return true;
		}else
		{
			return false;
		}
	}


public:
    RRTPathFinder();
    ~RRTPathFinder();
	void initParam(float search_radius, float th_stdev, float eta, int rrt_iter_count, Eigen::Vector3d car_size, ros::NodeHandle &nh, bool is_vis);
	void pathSearch(octomap::point3d start_pt, octomap::point3d end_pt);

	void updateMap(shared_ptr<octomap::OcTree> map);
    bool validGround(octomap::point3d query_point, float th_stdev);
	bool validGround(octomap::point3d query_point, float& mean_, float th_stdev);

    bool getPath(vector<octomap::point3d> &path);
	void getVisNode(vector<octomap::point3d> &vis_new_pt_set, 
					vector<octomap::point3d> &vis_block_pt_set, 
					vector<pair<octomap::point3d, octomap::point3d> > &vis_rrt_tree_set);
	bool getSearchState();


	

};

RRTPathFinder::RRTPathFinder()
{
    rrt_count_th_ = 40000;
    eta_ = 2.0;
    search_r_ = 4.0;
	th_stdev_ = 0.1;

    	// load car size
	carBodySize(0) = 0.6;  //x
	carBodySize(1) = 0.8;  //y
	carBodySize(2) = 0.8;  //z  
	
	search_Finished_ = false;
}

void RRTPathFinder::initParam(float search_radius, float th_stdev, float eta, int rrt_iter_count, Eigen::Vector3d car_size, ros::NodeHandle &nh, bool is_vis){
	search_r_ = search_radius;
	th_stdev_ = th_stdev;
	eta_ = eta;
	rrt_count_th_ = rrt_iter_count;

	carBodySize(0) = car_size(0);
	carBodySize(1) = car_size(1);
	carBodySize(2) = car_size(2);

	// n_ = nh;
	// is_vis_ = is_vis;

	// vis_pub_ = n_.advertise<visualization_msgs::Marker>( "rrt_sample_node", 10 ); // 发布采样点
	// rrt_tree_pub_ = n_.advertise<visualization_msgs::Marker>("rrt_tree", 10); //发布rrt 树
    // rrt_path_pub_ = n_.advertise<visualization_msgs::Marker>("rrt_path", 10);	//发布回溯rrt节点

}


RRTPathFinder::~RRTPathFinder()
{

}

void RRTPathFinder::pathSearch(octomap::point3d start_pt, octomap::point3d end_pt){
    
	float mean;
    int checking;
	search_Finished_ = false;

	start_pt_ = start_pt;
    end_pt_ = end_pt;

	start_pt_(2) = 5;
	if(validGround(start_pt_, mean, th_stdev_)){
		start_pt_(2) = mean + carBodySize(2);
	}else
	{
		start_pt_(2) = start_pt(2);
	}

	end_pt_(2) = 5;
	if(validGround(end_pt_, mean, th_stdev_)){
		end_pt_(2) = mean + carBodySize(2);
	}else
	{
		end_pt_(2) = end_pt(2);
	}


    rrt_count_ = -1;

	vis_rrt_tree_set_.clear();
	vis_new_pt_set_.clear();
	vis_block_pt_set_.clear();

    rrt_path_.clear();
    rrt_tree_set_.clear();
    rrtNode start_node = {start_pt_, 0.0};
    rrt_tree_set_.insert(make_pair(start_pt_, start_node));

 
    double minX, minY, minZ, maxX, maxY, maxZ;
	map_tree_->getMetricMin(minX, minY, minZ);
	map_tree_->getMetricMax(maxX, maxY, maxZ);


    // rrt查找过程
    while (rrt_count_ <= rrt_count_th_)
    {
		rrt_count_++;

        // if(rrt_count_ % 30000 == 0)
        // {
		// 	cout <<"[rrt path finder]: input any to continue: " << endl;
        //     int a = cin.get();  
        // }

		#ifdef DEBUG
        cout << "[rrt path finder]-----------------------"<< endl;
		cout << "[rrt path finder]: rand smaple a point " << endl;
		#endif

        if (rrt_count_ % 8 == 0)
		{
			rand_pt_(0) = getRandData(end_pt_(0) - 5, end_pt_(0) + 5);
			rand_pt_(1) = getRandData(end_pt_(1) - 5, end_pt_(1) + 5);
		}else
		{
			rand_pt_(0) = getRandData(minX, maxX);
			rand_pt_(1) = getRandData(minY, maxY);
		}

		#ifdef DEBUG
		cout << "[rrt path finder]: rand_pt_ " << rand_pt_(0) <<" " << rand_pt_(1) << " " << rand_pt_(2) << endl;
		#endif

        nearest_pt_ = Near(rrt_tree_set_, rand_pt_);

		#ifdef DEBUG
		cout << "[rrt path finder]: nearest_pt_ " << nearest_pt_(0) <<" " << nearest_pt_(1) << " " << nearest_pt_(2) << endl;
		#endif

        new_pt_ = Steer(nearest_pt_, rand_pt_, eta_);
		#ifdef DEBUG
		cout << "[rrt path finder]: new_pt_ " << new_pt_(0) <<" " << new_pt_(1) << " " << new_pt_(2) << endl;
		#endif

        new_pt_(2) = 5;
		// cout << validGround(p_new, mean) << endl;
		if(validGround(new_pt_, mean, th_stdev_)){
			new_pt_(2) = mean + carBodySize(2);
		}else
		{
			new_pt_(2) = nearest_pt_(2);
		}

		#ifdef DEBUG
		cout << "[rrt path finder]: new_pt_ after " << new_pt_(0) <<" " << new_pt_(1) << " " << new_pt_(2) << endl;
		#endif
		
        checking = collisionFree(nearest_pt_, new_pt_);

		#ifdef DEBUG
		cout << "checking..." << checking << endl;
		#endif

        if (checking == 0)
        {
			#ifdef DEBUG
            cout<<"[rrt path finder]: collosion" << endl;
			#endif

			vis_block_pt_set_.push_back(new_pt_); // 阻断的节点, 用于显示
        }

        if (checking == 1)
        {
			#ifdef DEBUG
            cout <<"[rrt path finder]: rrt tree add new point " <<endl;
			#endif

			vis_new_pt_set_.push_back(new_pt_); // 加入八叉树的节点, 用于显示
			vis_rrt_tree_set_.push_back(make_pair(new_pt_, nearest_pt_)); //八叉树，用于显示

            // rrt-star
            rewirteTree(rrt_tree_set_, new_pt_, nearest_pt_, search_r_);

			// rrt
			// this_node_.father_node = nearest_pt_;
			// this_node_.cost = rrt_tree_set_.at(nearest_pt_).cost + Norm(nearest_pt_, new_pt_);
			// rrt_tree_set_.insert(make_pair(new_pt_, this_node_));

			if (isGoal(new_pt_, end_pt_))
			{
				this_node_.father_node = new_pt_;
				this_node_.cost = rrt_tree_set_.at(new_pt_).cost + Norm(new_pt_, end_pt_);
				rrt_tree_set_.insert(make_pair(end_pt_, this_node_));

				backTree(end_pt_, start_pt_, rrt_path_);
				reverse(rrt_path_.begin(), rrt_path_.end()); //反转vector
				search_Finished_ = true;
				cout <<"[rrt path finder]: rrt have got goal" << endl;
				break;

			}
			
        }
    
    }

	// cout <<"[rrt path finder]: NO path finded" << endl;
    
}


// 遍历rrt, 返回查询节点到起始节点的所有父节点, 以及查询节点代价
void RRTPathFinder::backTree(octomap::point3d p, octomap::point3d p_init, vector<octomap::point3d> &rrt_tree){
	octomap::point3d point_back;
	point_back = p;
	for (size_t i = 0; i < rrt_tree_set_.size(); i++)
	{
		auto it = rrt_tree_set_.find(point_back);
		if (it != rrt_tree_set_.end())
		{
			rrt_tree.push_back(point_back);
			point_back = it->second.father_node;

			if (point_equal(point_back, p_init))
			{
				rrt_tree.push_back(point_back);
				return;
			}
			
		}else
		{
			cout << "[ERROR]: back point not in rrt tree!!!!" << endl;
			return;
		}
	}	
}


bool RRTPathFinder::isGoal(octomap::point3d new_point, octomap::point3d end_point){
    if(Norm(new_point, end_point) < map_resolution_*6)
    {
        return true;
    }else
    {
        return false;
    }
    
}

// 地面状态,  0: 无地面信息　1:有地面信息
int RRTPathFinder::collisionFree(octomap::point3d nearest_point, octomap::point3d new_point){
	float step_length = map_resolution_*3; //*2
	int step_nu = ceil(Norm(nearest_point, new_point)/step_length);  //ceil(x)返回的是大于x的最小整数
	octomap::point3d step_point = nearest_point;
	int state;
	for (size_t i = 0; i < step_nu; i++)
	{
		step_point = Steer(step_point, new_point, step_length);
		if(validGround(step_point, th_stdev_)){
			state = 1;
			continue;
		}else
		{
			state = 0;
			return state;
		}
	}
	return state;
	
}


// process octomap
bool RRTPathFinder::validGround(octomap::point3d query_point, float th_stdev){
	// sreach the ground state
	// sreach pattern
	//     *
	//   *****
	//  ***#***
	//   *****
	//     * 
	vector <octomap::point3d> sreach_area;
	sreach_area.push_back(query_point);
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution_, query_point(1)                    , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1)                    , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution_, query_point(1)                    , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1)                    , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 3*map_resolution_, query_point(1)                    , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 3*map_resolution_, query_point(1)                    , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                    , query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                    , query_point(1) + 2*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                    , query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                    , query_point(1) - 2*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));

	octomap::point3d direction(0., 0., -1.0); //查询方向: z 负方向
	vector<octomap::point3d> ground_point_set;
	octomap::point3d ground_point;
	float point_z_sum=0.0;
	bool get_ground;

	for (auto it=sreach_area.begin(); it != sreach_area.end(); it++)
	{	
		get_ground = map_tree_->castRay(*it, direction, ground_point, true, 10.0);
		// cout << "search " << *it << endl;
		if(!get_ground){
			continue;
		}else
		{
			ground_point_set.push_back(ground_point);
			point_z_sum += ground_point(2);
		}
	}

	if(ground_point_set.size() < 3){
		return false;
	}else
	{
		float mean = point_z_sum / ground_point_set.size();
		double accum  = 0.0;
		for(auto it=ground_point_set.begin(); it != ground_point_set.end(); it++){
			accum += ((*it)(2) - mean)*((*it)(2) - mean);
		}
		float stdev = sqrt(accum/(ground_point_set.size()-1));
		if(stdev > th_stdev){ //0.005 0.1
			// cout<< query_point <<  " stdev: " << stdev <<endl; 
			return false;
		}
	}
	return true;

}


// 返回地形高度，用于显示
bool RRTPathFinder::validGround(octomap::point3d query_point, float& mean_, float th_stdev){
	// sreach the ground state
	// sreach pattern
	// 
	//   *****
	//   **#**
	//   *****
	//
	     
	vector <octomap::point3d> sreach_area;
	sreach_area.push_back(query_point);
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution_, query_point(1)                    , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1)                    , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution_, query_point(1)                    , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1)                    , query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0) + 3*map_resolution_, query_point(1)                   , query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0) - 3*map_resolution_, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                    , query_point(1) + 1*map_resolution_, query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) + 2*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                    , query_point(1) - 1*map_resolution_, query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) - 2*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));

	octomap::point3d direction(0., 0., -1.0); //查询方向: z 负方向
	vector<octomap::point3d> ground_point_set;
	octomap::point3d ground_point;
	float point_z_sum=0.0;
	bool get_ground;

	if (map_tree_->castRay(query_point, direction, ground_point, true, 10.0))
	{
		mean_ = ground_point(2);
		// cout << "mean detect: "<< mean_ << endl;
		return true;
	}
	
	for (auto it=sreach_area.begin(); it != sreach_area.end(); it++)
	{	
		get_ground = map_tree_->castRay(*it, direction, ground_point, true, 10.0);
		if(!get_ground){
			continue;
		}else
		{
			ground_point_set.push_back(ground_point);
			point_z_sum += ground_point(2);
		}
	}

	if(ground_point_set.size() < 3){
		return false;
	}else
	{
		mean_ = point_z_sum / ground_point_set.size();
		// cout << "mean detect: "<< mean_ << endl;
		return true;
	}
	
}

void RRTPathFinder::rewirteTree(Point_Pair_Set &V, octomap::point3d new_point, octomap::point3d nearest_point, float search_r){
    // 注意此时 new_point还未加入到树中; nearest_point是树中的节点
    vector<octomap::point3d> potential_parent;
    multimap<float, octomap::point3d> cost_point_pair;  //键值不能重复，相同cost如何处理-> 使用multimap //代价-节点 //升序排列
    float min_cost, dis_cost;
    float dis;
    int check;

    auto it1 = V.find(nearest_point);
    if (it1 != V.end())
    {
        min_cost = it1->second.cost + Norm(nearest_point, new_point);
    }else
    {
        cout << "[ERROR]: nearest_point not in rrt tree!!!!" << endl;
        return;
    }

	// 遍历
    for (TreeIterator iter = V.begin(); iter != V.end(); iter++)
    {
        dis = Norm(iter->first, new_point);
        if(dis <=search_r )
        {
            dis_cost = dis + iter->second.cost;
            potential_parent.push_back(iter->first);
			cost_point_pair.insert(make_pair(dis_cost, iter->first));
        }
    }

    //choose parent 排序处理
    //为什么还要排序，而不是取第一个元素？  因为第一个元素虽然cost最小，但是并不一定collisionFree
    for (multimap<float, octomap::point3d>::iterator it = cost_point_pair.begin(); it != cost_point_pair.end(); it++){
        dis_cost = (*it).first;

        if (dis_cost == min_cost)
		{
            this_node_.father_node = nearest_point;
            this_node_.cost = dis_cost;
			V.insert(make_pair(new_point, this_node_));
			return;
		}

        if (dis_cost < min_cost)
        {
            check = collisionFree((*it).second, new_point);
            if (check == 0)
			{
				continue;
			}
            if (check == 1)
            {
                this_node_.father_node = (*it).second;
                this_node_.cost = dis_cost;
                V.insert(make_pair(new_point, this_node_));
                for (auto iter1 = cost_point_pair.begin(); iter1 != cost_point_pair.end(); iter1++)
                {
                    if(dis_cost + Norm((*iter1).second, new_point) < (*iter1).first - Norm((*iter1).second, new_point)){
                        auto n = V.erase((*iter1).second);
                        if (n = 0)
                        {
                            cout << "[ERROR]: not erase point not in rrt tree!!!!" << endl;
                        }
                        this_node_.father_node = new_point;
                        this_node_.cost = dis_cost + Norm((*iter1).second, new_point);
                        V.insert(make_pair((*iter1).second, this_node_));
                        
                    }

                }

                return;
            }
            
        }
        
    }

}

octomap::point3d RRTPathFinder::Steer(octomap::point3d nearest_point, octomap::point3d rand_point, float eta){
    octomap::point3d new_point;
    float dis = Norm(nearest_point, rand_point);
    if(dis <= eta)
    {
        new_point = rand_point;
    }else{
        new_point(0) = nearest_point(0) + eta * (rand_point(0) - nearest_point(0)) / dis;
        new_point(1) = nearest_point(1) + eta * (rand_point(1) - nearest_point(1)) / dis;
    }

    return new_point;
}

// 计算rrt_tree上离随机点最近的点
octomap::point3d RRTPathFinder::Near(Point_Pair_Set &V , octomap::point3d p){
	float min = Norm(start_pt_, p);
    float temp;
    octomap::point3d nearest_point;

	for (auto &it : V)
	{
		temp = Norm(it.first, p);
		if (temp <= min)
		{
			min = temp;
			nearest_point = it.first;
		}	
	}
	

    return nearest_point;
}


void RRTPathFinder::updateMap(shared_ptr<octomap::OcTree> map){
		map_tree_ = map;
		map_resolution_ = map_tree_->getResolution();
		cout<<"[Path Finder]map Resolution: "<< map_resolution_ <<endl;
}


bool RRTPathFinder::getPath(vector<octomap::point3d> &path){
	if (search_Finished_)
	{
		path = rrt_path_;
		return true;
	}else
	{
		cout << "[rrt path finder] finding......... Pealse wait" <<endl;
		return false;
	}
}

void RRTPathFinder::getVisNode(vector<octomap::point3d> &vis_new_pt_set, 
					vector<octomap::point3d> &vis_block_pt_set, 
					vector<pair<octomap::point3d, octomap::point3d> > &vis_rrt_tree_set){

						vis_new_pt_set = vis_new_pt_set_;
						vis_new_pt_set_.clear();

						vis_block_pt_set = vis_block_pt_set_;
						vis_block_pt_set_.clear();

						vis_rrt_tree_set = vis_rrt_tree_set_;
						vis_rrt_tree_set_.clear();
							
					}

bool RRTPathFinder::getSearchState(){
	return search_Finished_;
}