#!/bin/bash
#cd /home/ubuntu/contiki-master-rpl/examples/ipv6/rpl-collect
echo "enter node id start number:"
read A
echo "enter number of new chip that needed allocate NODEID"
read B
echo $A,$B
if echo "$A" | grep -q '^[0-9]\+$' ;then
	if echo "$B" | grep -q '^[0-9]\+$' ;then
		let C="$A+$B-1"
		echo "creat $B's new id form $A - $C "
	else
		echo "wrong input"
		exit
	fi
fi
rm -rf /home/ubuntu/NewNode
mkdir  /home/ubuntu/NewNode
echo $C
for x in $(seq $A $C)
do
	d=`echo "obase=16;$x"|bc`
        make TARGET=trxeb1120 clean
	make TARGET=trxeb1120 BURN_NODEID=1 NODEID=$x $1
        mv $1 $d.hex
	mv $d.hex  /home/ubuntu/NewNode
done

rm *.hex
make clean
echo "all done"
