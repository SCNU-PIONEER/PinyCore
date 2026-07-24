# 滑模变结构控制 v1.0

目前上车的这一版代码是用的一个非常简单的仅用位移误差去做的控制器，截止至7.21，就目前调过的两辆车——26区域赛英雄和牢全来看，该控制器在大转动惯量的情况不论是在参数整定的复杂度还是控制效果都是优于小转动惯量系统的，相比于pid双环控制器，这套滑模控制器需要整定的参数非常少：①c滑模相平面斜率②epsilon滑模等速趋近参数③k指数趋近参数。但它也有明显的缺点，一旦参数整定得不好，那么就会非常非常得抖，抖震几乎是滑模控制器的一大特性，建议是在结果输出的时候加上滤波。

<figure>
    <img src="26EagleHeroTest.png" alt="26英雄实测" style="width: 1000px; display: block; margin: 0 auto;">
</figure>
可以看到，在26区域赛英雄上跟得还是不错的

## 使用方法（供参考）
### 实例化加初始化
<ReachingSmc>选择需要的趋近律子类 <c>相平面斜率 <ε>等速趋近参数 <k>指数趋近参数 <α>幂次趋近参数 <dt>采样周期
static SMC::<ReachSmc> yawSmcCtrl(<c>0.1f, <ε>0.2f, <k>0.1f, <dt>1.f / 1000.f);

### 创建控制函数
void Standard::yawImuSmcCtrl()
{
    float yawSmcTorq = yawSmcCtrl.smcSimplePosCalc(PINYMOTOR::getMinorArc(cmdImuYaw_, yawState_.pos));
    yaw()->cmdTorq(imuLpf[1].process(yawSmcTorq));
}
