这个包有两个节点, 分别是

```shell
contrast_node // 对比度画面
threshold_node // 二值化画面
```

如果要使用, 请输入

```shell
rosrun threshold threshold_node
or
rosrun threshold contrast_node
```

如果找不到threshold这个包, 请前往 `工作目录/devel/lib/threshold` 下直接运行可执行文件



#### 在 apriltag_ros中使用画面

将 

```
<remap from="image_rect" to="xxxxxxxx" />
这行代码改成
<remap from="image_rect" to="/contrast/image" />  可以使用对比度画面
改成
<remap from="image_rect" to="/threshold/image" />  可以使用二值化画面
```

然后在rqt中寻找 dynamic reconfigure 在threshold话题中可以动态调节画面的 **二值化阈值** 或者

**对比度** , 对比度的亮度没调过, 对比度建议调为1.8-2.5 或者你自己试试!