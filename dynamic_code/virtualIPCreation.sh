nextip(){
    IP=$1
    IP_HEX=$(printf '%.2X%.2X%.2X%.2X\n' `echo $IP | sed -e 's/\./ /g'`)
    NEXT_IP_HEX=$(printf %.8X `echo $(( 0x$IP_HEX + 1 ))`)
    NEXT_IP=$(printf '%d.%d.%d.%d\n' `echo $NEXT_IP_HEX | sed -r 's/(..)/0x\1 /g'`)
    echo "$NEXT_IP"
}

echo -n "Enter the number of virtual IP addresses to be created: "
read NUM
NUM=$((NUM-1))

echo -n "Enter the interface name of the ip address: "
read interface

echo -n "Enter the ip address of the mission planner: "
read missionPlannerIP

IP=$(hostname -I)


for i in $(seq 0 $NUM); do
    IP=$(nextip $IP)
    if [ $IP == $missionPlannerIP ]
    then
        IP=$(nextip $IP)
    fi       
    $(sudo ifconfig $interface:$i $IP netmask 255.255.255.0 up)
done

NUMIP=$((NUM+1))
echo -n "$NUMIP virtual ip Addresses created successfully"
printf "\n"