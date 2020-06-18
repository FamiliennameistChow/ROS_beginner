gazebo场景与模型使用说明
=====

## 场景
* compeition.world使用说明  
  依赖`models/group_A`

* moon.world使用说明  
  依赖`models/Amy_terrain`, 将其放在`~/.gazebo/models/`下

## 模型

* Amy_terrain下修改场景

```xml
<?xml version="1.0"?>

<model>
  <name>Amy_terrain</name>
  <version>1.0</version>
  <!-- <sdf version="1.6">model_flater.sdf</sdf> -->  <!--有山的地表场景-->
  <sdf version="1.4">model.sdf</sdf> <!--有坑的地表场景-->

  <author>
    <name>chow/chen</name>
  </author>

  <description>
    A self_made heightmap. 
  </description>

</model>
```