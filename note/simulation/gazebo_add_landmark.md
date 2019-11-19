# gazebo_添加降落地标

----

[参考](https://blog.csdn.net/zyh821351004/article/details/49785855)  

本教程文件保存在[mark_label](https://github.com/FamiliennameistChow/ROS_beginner/blob/master/gazebo/models)


----

首先在home目录下按`ctrl+h`打开隐藏文件
**gazebo模型的存放位置`~/.gazebo/models/`**  

新建一个模型文件夹`mark_label`

我们将在该文件夹中构建如下目录结构文件:

```
model.sdf

model.config

materials 文件夹
   --scripts  文件夹
       mark_label.material     纹理信息
   --textures  文件夹
       h.png  地标图像
       x.png  地标图像
```

----


进入`mark_label`

## 创建`model.sdf`

```xml
<?xml version='1.0'?>
<sdf version='1.4'>
  <model name="mark_label">
    <link name='link'>
      <pose>0 0 0.115 0 0 0</pose>
      <inertial>
        <mass>0.390</mass>
        <inertia>
          <ixx>0.00058</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00058</iyy>
          <iyz>0</iyz>
          <izz>0.00019</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
                <box>
                  <size>.496 .496 .01</size>
                </box>
        </geometry>
      </collision>

      <visual name='visual'>
        <geometry>
                <box>
                  <size>.496 .496 .01</size>
                </box>
        </geometry>

        <material>
          <script>
            <uri>model://mark_label/materials/scripts</uri>
            <uri>model://mark_label/materials/textures</uri>
            <name>Mark/Diffuse</name>
          </script>
        </material>
      </visual>
    </link>     
  </model>
</sdf>


```

说明：

```xml
        <material>
          <script>
            <uri>model://mark_label/materials/scripts</uri>　纹理信息配置文件路径
            <uri>model://mark_label/materials/textures</uri>　纹理文件路径
            <name>Mark/Diffuse</name>
          </script>
        </material>
```
注意: `<name>Mark/Diffuse</name>` 需要与后面的`mark_label.material`中`material Mark/Diffuse`一致

## 创建`model.config`

```xml
<?xml version="1.0"?>

<model>
  <name>Mark_label</name>
  <version>1.0</version>
  <sdf version="1.4">model.sdf</sdf>

  <author>
    <name>chow</name>
  </author>

  <description>
    Landing Mark
  </description>
</model>
```


**注意:** 其中这里`<name>Mark_label</name>`中的`Mark_label`将会是你`gazebo`中该模型的名字


## 创建`mark_label.material`

```
material Mark/Diffuse
{
	receive_shadows off
	technique
	{
		pass
		{
			texture_unit
			{
				
				filtering anistropic
				max_anisotropy 16
			}
		}
	}
}
```

注意该文件是区分`tab`与`空格`的  
这里的`material Mark/Diffuse`需要与`model.sdf`中`<name>Mark/Diffuse</name>`一致　　
修改`texture h.png`的图片名字可以修改贴图


## 使用

打开gazebo

在`insert`中可以找到刚刚建立的模型`Mark_label`

**注意:第一次加载可能会很慢**

![使用](_v_images/20191118185621739_975048952.png)