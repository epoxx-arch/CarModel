# RSS model 

## 论文记录

### 功能安全与名义安全

*功能安全*：硬、软件系统功能的完整性。  ISO 26262




## 1.1 纵向安全距离

```cpp
double v_r = ; // 后车速度 
double rou = ; // 反应时间 
double alpha_max = ;   // 最小刹车加速度
double beta_min =  ; // 最大刹车减速度
double v_f =  ; // 前车速度 
double beta_max = ; // 最小刹车减速度

double d_min_1 = v_r * rou;
double d_min_2 = 0.5 * alpha_max * rou * rou; 
double d_min_3 = std::power(v_r + rou * alpha_max,2) / beta_min * 0.5;
double d_min_4 = - v_f * v_f / beta_max * 0.5;

double d_min = d_min_1 + d_min_2 + d_min_3 + d_min_4;
```


## 1.2 横向安全距离 
```cpp 
double mu = ;
double v1 = ;
double v1_rou = ;
double rou = ;
double beta_1_lat_min = ;
double v_2 = ;
double v_1_p = ;
double v_2_p = ;
double beta_2_lat_min = ;

double d_min_1 = mu;
double d_min_2 = (v1 + v1_rou) * 0.5;
double d_min_3 = v_1_p * v_1_p / beta_2_lat_min * 0.5; 
double d_min_4 = -(v_2+ v_1_p) * 0.5 * rou - v_2_p * v_2_p / beta_2_lat_min * 0.5;
double d_min = d_min_1 + d_min_2 + d_min_3 + d_min_4;
```

## 
