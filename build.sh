#!/bin/bash

>build.log
mkdir build && pushd build

echo -e "\nsrc package bash-3.0.16.tar.gz..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar zxvf ../src/bash-3.0.16.tar.gz
pushd bash-3.0.16
./configure | tee -a ../../build.log
make | tee -a ../../build.log
popd

echo -e "\nsrc package busybox-1.17.4.tar.bz2..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar jxvf ../src/busybox-1.17.4.tar.bz2
pushd busybox-1.17.4
tar zxvf ../../src/579.11.123/busybox-1.17.4-local-patch.tgz
cp -rf ipa-diff/* .
make oldconfig | tee -a ../../build.log
make | tee -a ../../build.log
popd

echo -e "\nsrc package cramfs-1.1.tar.gz..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar zxvf ../src/cramfs-1.1.tar.gz
pushd cramfs-1.1/
tar zxvf ../../src/579.11.123/cramfs-1.1-local-patch.tar.gz
cp -rf ipa-diff/* .
export OBJDIR=$PWD
make -f GNUmakefile | tee -a ../../build.log
popd

echo -e "\nsrc package u-boot-1.3.4.tar.bz2..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar jxvf ../src/u-boot-1.3.4.tar.bz2
pushd u-boot-1.3.4
cp ../../src/579.11.123/u-boot-1.3.4-local-patch-224.gz .
cp ../../src/579.11.123/u-boot-1.3.4-picochip-3.2.4-patch.gz .
gunzip *.gz
patch -p1 <u-boot-1.3.4-picochip-3.2.4-patch
patch -p1 <u-boot-1.3.4-local-patch-224
export TYPE=xc && make ipaccessip202ff_config | tee -a ../../build.log
cat >>common/main.c<<EOF
unsigned long long get_ticks(void)
{
       return get_timer(0);
}
ulong get_tbclk (void)
{
       ulong tbclk;

       tbclk = CFG_HZ;
       return tbclk;
}
int raise() { return 0; }
EOF
make | tee -a ../../build.log
popd

echo -e "\nsrc package strongswan-4.2.12.tar.gz..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar zxvf ../src/strongswan-4.2.12.tar.gz
pushd strongswan-4.2.12
tar zxvf ../../src/579.11.123/strongswan-4.2.12-official-patch.tar.gz
patch -p1 <patches_strongswan/04_swapped_ts_check_patch/strongswan-4.x.x._swapped_ts_check.patch
patch -p1 <patches_strongswan/03_invalid_ike_state_patch/strongswan-4.x.x_invalid_ike_state.patch
./configure | tee -a ../../build.log
make | tee -a ../../build.log
popd

echo -e "\nsrc package ethtool-6.tar.gz..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar zxvf ../src/ethtool-6.tar.gz
pushd ethtool-6
./configure | tee -a ../../build.log
make | tee -a ../../build.log
popd

echo -e "\nsrc package gcc-2008q3-72.tar.bz2..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar jxvf ../src/gcc-2008q3-72.tar.bz2
pushd gcc-4.3
mkdir build
pushd build
../configure --with-mpfr=/usr/local --with-gmp=/usr/local --disable-multilib | tee -a ../../../build.log 
make | tee -a ../../../build.log 
popd
popd

echo -e "\nsrc package glibc-2008q3-72.zip..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
unzip ../src/glibc-2008q3-72.zip
pushd glibc-2008q3-72/
tar jxvf glibc-2008q3-72.tar.bz2
pushd glibc-2.8/
cp ../../../src/579.11.123/glibc-2008q3-72-ipa.patch.gz .
gunzip glibc-2008q3-72-ipa.patch.gz 
patch -p1 <glibc-2008q3-72-ipa.patch
mkdir build && pushd build
../configure --disable-sanity-checks | tee -a ../../../../build.log
make | tee -a ../../../../build.log
popd
popd
popd

echo -e "\nsrc package iproute2-2.6.26.tar.bz2..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar jxvf ../src/iproute2-2.6.26.tar.bz2
pushd iproute2-2.6.26
tar zxvf ../../src/579.11.123/iproute2-2.6.26-local-patch.tar.gz
cp -rf patches/* .
./configure | tee -a ../../build.log
make | tee -a ../../build.log
popd

echo -e "\nsrc package gmp-4.2.1.tar.gz..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar zxvf ../src/gmp-4.2.1.tar.gz
pushd gmp-4.2.1
./configure | tee -a ../../build.log
make | tee -a ../../build.log
popd

echo -e "\nsrc package linux-2.6.28.tar.bz2..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar jxvf ../src/linux-2.6.28.tar.bz2
pushd linux-2.6.28/
cp ../../src/579.11.123/linux* .
gunzip *.gz
cp ../../src/linux-2.6.28-dot-config .config
patch -p1 <linux-v2.6.28-picochip-3.2.4-patch
patch -p1 <linux-2.6.28-local-patch-224
# choose defaults
make oldconfig
make | tee -a ../../build.log
popd
echo -e "\nsrc package iptables-1.4.2.tar.bz2..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar jxvf ../src/iptables-1.4.2.tar.bz2
pushd iptables-1.4.2
tar zxvf ../../src/579.11.123/iptables-1.4.2-local-patch.tar.gz
cp -rf patches/* .
./configure | tee -a ../../build.log
make | tee -a ../../build.log
popd

echo -e "\nsrc package mtd-utils.tgz..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar zxvf ../src/mtd-utils.tgz
pushd mtd-utils-1.0.0
make | tee -a ../../build.log
popd

echo -e "\nsrc package procps-3.2.7.tar.gz..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar zxvf ../src/procps-3.2.7.tar.gz
pushd procps-3.2.7
make | tee -a ../../build.log
popd

echo -e "\nsrc package smartmontools-5.37.tar.gz..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar zxvf ../src/smartmontools-5.37.tar.gz
pushd smartmontools-5.37
./configure | tee -a ../../build.log
make | tee -a ../../build.log
popd

echo -e "\nsrc package i2c-tools-3.0.2.tar.bz2..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar jxvf ../src/i2c-tools-3.0.2.tar.bz2 
pushd i2c-tools-3.0.2
make | tee -a ../../build.log
popd

echo -e "\nsrc package ipkg-0.99.163.tar.gz..." | tee -a ../build.log
echo "________________________________________________________________________________" | tee -a ../build.log
tar zxvf ../src/ipkg-0.99.163.tar.gz
pushd ipkg-0.99.163
./configure | tee -a ../../build.log
make | tee -a ../../build.log
popd

