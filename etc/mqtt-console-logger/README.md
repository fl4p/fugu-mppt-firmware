
this data logger stores log entries from all chargers on the network through MQTT.
it uses rotating log files.

to enable compression:

```
bash
apt install btrfs-progs btrfs-compsize attr
fallocate -l 1G /var/log_btrfs.img
mkfs.btrfs /var/log_btrfs.img
losetup --find --show /var/log_btrfs.img
mkdir /var/log_btrfs
comp=zstd:3 #zlib,lzo
mount -o compress=$comp /dev/loop0 /var/log_btrfs
btrfs property set /var/log_btrfs compression $comp
touch /var/log_btrfs/test
getfattr --match=- /var/log_btrfs/test
compsize /var/log_btrfs/fugu_

```

fstab:
```

```

```
lzo         37%      784K         2.0M         2.0M       
zlib        24%      528K         2.0M         2.0M       
```
