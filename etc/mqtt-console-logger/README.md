this data logger stores log entries from all chargers on the network through MQTT.
it uses rotating log files, each file is limited to 10 MB size and 10 files totally.

the stored data will have a lot of redundancy.
we can create a block device from a file with btrfs and enable zstd compression:

```
sudo bash
apt install btrfs-progs btrfs-compsize attr
fallocate -l 32M /var/log_btrfs.img
mkfs.btrfs /var/log_btrfs.img
losetup --find --show /var/log_btrfs.img
mkdir /var/log_btrfs
comp=zstd:3 #zlib,lzo
mount -o compress=$comp /dev/loop0 /var/log_btrfs
btrfs property set /var/log_btrfs compression $comp
cp fugu_console.log /var/log_btrfs/
getfattr --match=- /var/log_btrfs/fugu_console.log
compsize /var/log_btrfs/fugu_console.log
```

/etc/fstab:

```
/dev/loop0	/var/log_btrfs btrfs	compress=zstd:3 0	2
```

compression ratio:

```
lzo         37%      784K         2.0M         2.0M       
zlib        24%      528K         2.0M         2.0M       
```
