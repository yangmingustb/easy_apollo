
查看进程：

```
ps -aux
```

查看设备：

```
ls /dev/tty*
```



无法读取串口消息，可以增加权限：
```
sudo chmod 777 /dev/ttyUSB0
```


无法读取到can消息，可以使用linuxcan/example的测试工具检测以下，检测读到几个kavasar通道。
必要的时候，重新安装一下linuxcan。
