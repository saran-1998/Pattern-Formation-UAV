nextip(){
    IP=$1
    IP_HEX=$(printf '%.2X%.2X%.2X%.2X\n' `echo $IP | sed -e 's/\./ /g'`)
    NEXT_IP_HEX=$(printf %.8X `echo $(( 0x$IP_HEX + 1 ))`)
    NEXT_IP=$(printf '%d.%d.%d.%d\n' `echo $NEXT_IP_HEX | sed -r 's/(..)/0x\1 /g'`)
    echo "$NEXT_IP"
}
dronekitPort=5760
UDPPort=14551
echo -n "Enter the no of drones: "
read noOfDrones
missionPlannerPort=$((UDPPort+noOfDrones))
echo -n "Enter the IP address of Mission Planner: "
read missionPlannerIPAddress
noOfDronesLoopVariable=$((noOfDrones-1))
IP=($(hostname -I))
for i in $(seq 0 $noOfDronesLoopVariable); do
    indexValue=$((i+1))
    xterm -title "Dronekit $i" -hold -e dronekit-sitl copter3.0 -I$i &
    xterm -title "MavProxy $i" -hold -e mavproxy.py --master tcp:$IP:$dronekitPort --sitl 127.0.0.1:5501 --out $IP:$UDPPort --mav10 --source-system=$indexValue --out $missionPlannerIPAddress:$missionPlannerPort  &
    dronekitPort=$((dronekitPort+10))
    UDPPort=$((UDPPort+1))
    missionPlannerPort=$((missionPlannerPort+1))
    IP=$(nextip $IP)
done