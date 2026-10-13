# 黑色单腔拉链笔袋

![preview](three_quarter.jpg)

袋身闭合外形240×70×75mm，拉链行程216mm，圆环内径18mm/外径22mm（16边近似的内切直径17.65mm）。原点为底面中心，+X沿长度及开链方向，前面为−Y。包含圆环、拉头和白绳的实际闭合视觉包络约240×74×83.47mm。

第一章使用固定空腔袋身＋两片完整刚性袋唇近似，5刚体、4关节。先退完拉头再撑口，先合口再回拉；顺序由几何关键帧协调，无机械互锁，不模拟布料变形、解链力或保持力。

关节：`pencil_case.unzip`为沿+X的SLIDER，[0,0.216]m；前/后袋唇HINGE绕+X，[0,π/4]/[−π/4,0]rad；`pencil_case.ring_swing`为绕+Y的HINGE，[−π/2,π/2]rad。圆环绕上侧连接点在前面摆动。无禁碰对，装饰与链带随各自刚体，碰撞空腔和圆环孔保持开放。

全开碰撞净口约210×44.51mm，下半净腔230×64×51mm。22.5°部分撑口净宽约18.68mm，20mm粗笔不能从该姿态通过。整件放置选中组织根`pencil_case`，各刚体及约束锚点共同随动；各刚体内的语义部件不单独运动。第一章离散几何检查覆盖三笔、两橡皮、白短刀的指定入袋路径和装载合口，未验证夹爪空间、布料受力、Isaac拖拽/Play/Stop或机器人任务。

实际资产树（从object.blend导出，省略网格层）：

```text
pencil_case
  back_lip [rigid]
    cloth
    lining
    rim
    zipper_tape
  front_lip [rigid]
    cloth
    lining
    rim
    zipper_tape
  housing [rigid]
    binding
    label
    lining
    outer_cloth
  pull_ring [rigid]
    ring
  slider [rigid]
    attachment
    carriage
    white_cord
```
