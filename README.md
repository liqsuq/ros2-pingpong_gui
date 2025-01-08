# pingpong_gui
展示会デモ用に作ったROS2のsend/callback応答時間表示プログラム

# 要件
- UbuntuもしくはUbuntuベースのRedHawk Linux
- ROS2
- stress-ng

## リアルタイム設定(Jetson)
### RTスロットリングを無効化する
```
# cat > /etc/sysctl.d/10-sysctl-rt.conf << EOF
kernel.sched_rt_runtime_us=-1
EOF
```

### リアルタイム優先度及びメモリロックを一般ユーザーが使用できる様にする
(ファイル内のユーザー名は適宜変更する)
```
# cat > /etc/security/limits.d/limits_rt.conf << EOF
redhawk  -  memlock  unlimited
redhawk  -  rtprio   unlimited
EOF
```

### isolcpus等のリアルタイム用の起動パラメータを設定する
(コア設定はシステムとアプリケーションに応じて変更する)
```
# cat > /etc/default/gtub.d/grub-rt.cfg << EOF
GRUB_CMDLINE_LINUX="maxcpus=22 irqaffinity=0-17 rcu_nocbs=18-21 rcu_nocb_poll nohz_full=18-21 isolcpus=domain,managed_irq,18-21"
EOF
```

### (RedHawkの場合)shieldコマンドを一般ユーザーが使用できる様にする
```
# cat > /usr/share/polkit-1/actions/polkit_redhawk.policy << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE policyconfig PUBLIC
 "-//freedesktop//DTD polkit Policy Configuration 1.0//EN"
 "http://www.freedesktop.org/software/polkit/policyconfig-1.dtd">
<policyconfig>
  <action id="redhawk-shield">
    <description>Shield</description>
    <message>Privileges are required to shield processors.</message>
    <defaults>
      <allow_any>yes</allow_any>
      <allow_inactive>yes</allow_inactive>
      <allow_active>yes</allow_active>
    </defaults>
    <annotate key="org.freedesktop.policykit.exec.path">/usr/bin/shield</annotate>
  </action>
</policyconfig>
EOF
```

## Build
```
$ source /opt/ros/****/setup.bash
$ colcon build
```

## Execute

```
$ source install/setup.bash
$ ros2 run pendulum_gui pendulum_gui
```

```
$ source install/setup.bash
$ ros2 run pingpong_gui pingpong_gui
```
