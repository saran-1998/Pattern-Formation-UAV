nextip(){
    IP=$1
    IP_HEX=$(printf '%.2X%.2X%.2X%.2X\n' `echo $IP | sed -e 's/\./ /g'`)
    NEXT_IP_HEX=$(printf %.8X `echo $(( 0x$IP_HEX + 1 ))`)
    NEXT_IP=$(printf '%d.%d.%d.%d\n' `echo $NEXT_IP_HEX | sed -r 's/(..)/0x\1 /g'`)
    echo "$NEXT_IP"
}
IP=($(hostname -I))
echo -n "Enter the no of Followers:"
read noOfFollowers
noOfDrones=$((noOfFollowers+1))
UDPPort=14551
xterm -title "Leader Drone" -hold -e python formation_main_leader.py $IP $UDPPort $noOfDrones &
UDPPort=$((UDPPort+1))
IP=$(nextip $IP)
for i in $(seq 1 $noOfFollowers)
do
    xterm -title "Follower Drone $i" -hold -e python formation_main_follower.py $IP $UDPPort&
    UDPPort=$((UDPPort+1))
    IP=$(nextip $IP)
done