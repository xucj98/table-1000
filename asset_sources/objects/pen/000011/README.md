# 白身黑尾带盖笔

![preview](three_quarter.jpg)

白色笔杆、黑色尾端/肩环与深色印记，白色空心帽，闭合150×14×14mm。

两个独立完整刚体组件：笔身 `body` 与空心笔帽 `cap`；无永久关节。−X朝笔尖及拔帽方向。

## 资产树

从当前 `object.blend` 实际父子关系导出，省略视觉/碰撞网格。部件随所属刚体运动。

```text
body [rigid]
  barrel
  neck
  print
  rear_plug
  shoulder_band
  tip
cap [rigid]
  end
  lip
  shell
```
