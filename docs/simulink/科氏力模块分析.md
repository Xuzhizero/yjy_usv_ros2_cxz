# Simulink科氏力模块分析

## 提问

![Simulink Coriolis Module](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752656567864-07b85e5f-665f-4e12-b14d-bf2d0e74b937.png)

![Coriolis Module Details](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752656586521-967bdba3-9126-42b6-a726-83f556923042.png)

图中是Simulink的科氏力模块，告诉我，这个模块是否只包含了附加质量科氏力，还是说也包含了刚体科氏力？

## 回答

如你所见，这个 Coriolis 子系统里既有 "Compute CA" 又有 "Compute CRB" 两个子块：

1. **Compute CA**：根据附加质量矩阵 MA 和相对速度 ν_r 构造附加质量科氏矩阵 C_A。
2. **Compute CRB**：根据刚体惯性矩阵（hull+payload 在 CO）、质心偏移 H、角速度 [p,q,r] 等，构造刚体科氏矩阵 C_RB。
3. 然后用一个加法把两者相加：

$ C = C_{RB} + C_{A}\,, $

   再乘以速度向量，输出完整的科氏力向量。

所以，这个模块**并不**只是附加质量科氏力，而是将**刚体科氏**和**附加质量科氏**两部分都一起计算、相加，输出总的科氏力。
