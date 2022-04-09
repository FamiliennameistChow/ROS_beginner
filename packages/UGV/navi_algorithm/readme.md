# 这里是路径规划算法

----

## 前端路径搜索

### TODO List

- [x] rrt  
- [x] rrt-connet  
- [x] rrt-star  
- [ ] Kinodynamic rrt-star  
- [x] A-star  

### 基于采样的前端路径搜索算法

- rrt 

![rrt算法](./pic/2d-rrt.png)

- rrt-connet

![rrt-conet算法](./pic/2d-rrtConnet.png)

- rrt-star

![rrt-star算法](./pic/2d-rrtStar.png)
---

### 如何使用

- rrt算法测试
```sh
roslaunch navi_algorithm 2d_map_rrt.launch
```

- A-star

```
roslaunch navi_algorithm 2d_map_A_star.launch
```


## 后端轨迹优化

### TODO List

- [ ] 多项式轨迹表征  

- [ ] 贝塞尔曲线轨迹表征  

- [ ] B样条轨迹表征

- [ ] minimum-snap轨迹优化

- [ ] minimum-jerk轨迹优化



---

## 轨迹跟踪

### TODO List

- [ ] 纯轨迹跟踪
- [ ] 基于MPC的轨迹跟踪