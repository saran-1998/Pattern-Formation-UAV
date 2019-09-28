import math
import itertools

total_no_of_drones = int(input())

no_of_drones_per_edge = total_no_of_drones // 4
remainder_no_of_drones = total_no_of_drones % 4

edge1_angles = list()
edge1_displacements = list()
edge2_angles = list()
edge2_displacements = list()
edge3_angles = list()
edge3_displacements = list()
edge4_angles = list()
edge4_displacements = list()

if remainder_no_of_drones == 0:
    edge_length = 10 * no_of_drones_per_edge
    edge1_displacements = [
        i*10*1.414 for i in range(1, no_of_drones_per_edge+1)]
    edge1_angles = [135] * no_of_drones_per_edge
    edge2_displacements = [math.sqrt((edge_length ** 2)+(((no_of_drones_per_edge-i)*10) ** 2))
                           for i in range(1, no_of_drones_per_edge+1)]
    edge2_angles = [180 - int(math.degrees(math.atan2((no_of_drones_per_edge-i),no_of_drones_per_edge)))
                    for i in range(1, no_of_drones_per_edge+1)]
    edge3_displacements = [math.sqrt(
        (edge_length ** 2) + ((i*10) ** 2)) for i in range(1, no_of_drones_per_edge+1)]
    edge3_angles = [180 + int(math.degrees(math.atan2(i,no_of_drones_per_edge)))
                    for i in range(1, no_of_drones_per_edge+1)]
    edge4_displacements = [(no_of_drones_per_edge-i) *
                           10 * 1.414 for i in range(1, no_of_drones_per_edge)]
    edge4_angles = [225] * (no_of_drones_per_edge-1)
elif remainder_no_of_drones == 1:
    edge_length = 10 * (no_of_drones_per_edge + 1)
    edge1_displacements = [i * 1.414 * (edge_length/no_of_drones_per_edge)
                           for i in range(1, no_of_drones_per_edge+1)]
    edge1_angles = [135] * no_of_drones_per_edge
    edge2_displacements = [math.sqrt((edge_length ** 2) + (((no_of_drones_per_edge-i+1)*10) ** 2))
                           for i in range(1, no_of_drones_per_edge+2)]
    edge2_angles = [180 - int(math.degrees(math.atan2((no_of_drones_per_edge-i+1),(no_of_drones_per_edge+1))))
                    for i in range(1, no_of_drones_per_edge+2)]
    edge3_displacements = [math.sqrt((edge_length ** 2)+((i*(
        edge_length/no_of_drones_per_edge)) ** 2)) for i in range(1, no_of_drones_per_edge+1)]
    edge3_angles = [180 + int(math.degrees(math.atan2(i,no_of_drones_per_edge)))
                    for i in range(1, no_of_drones_per_edge+1)]
    edge4_displacements = [(no_of_drones_per_edge-i)*1.414*(edge_length/no_of_drones_per_edge)
                           for i in range(1, no_of_drones_per_edge)]
    edge4_angles = [225] * (no_of_drones_per_edge-1)
elif remainder_no_of_drones == 2:
    edge_length = 10 * (no_of_drones_per_edge + 1)
    edge1_displacements = [i * 1.414 * (edge_length/no_of_drones_per_edge)
                           for i in range(1, no_of_drones_per_edge+1)]
    edge1_angles = [135] * no_of_drones_per_edge
    edge2_displacements = [math.sqrt((edge_length ** 2) + (((no_of_drones_per_edge-i+1)*10) ** 2))
                           for i in range(1, no_of_drones_per_edge+2)]
    edge2_angles = [180 - int(math.degrees(math.atan2((no_of_drones_per_edge-i+1),(no_of_drones_per_edge+1))))
                    for i in range(1, no_of_drones_per_edge+2)]
    edge3_displacements = [math.sqrt(
        (edge_length ** 2)+((i*10) ** 2)) for i in range(1, no_of_drones_per_edge+2)]
    edge3_angles = [180 + int(math.degrees(math.atan2(i,(no_of_drones_per_edge+1))))
                    for i in range(1, no_of_drones_per_edge+2)]
    edge4_displacements = [(no_of_drones_per_edge-i)*1.414*(edge_length/no_of_drones_per_edge)
                           for i in range(1, no_of_drones_per_edge)]
    edge4_angles = [225] * (no_of_drones_per_edge-1)
else:
    edge_length = 10 * (no_of_drones_per_edge + 1)
    edge1_displacements = [i * 1.414 * (edge_length/no_of_drones_per_edge)
                           for i in range(1, no_of_drones_per_edge+1)]
    edge1_angles = [135] * no_of_drones_per_edge
    edge2_displacements = [math.sqrt((edge_length ** 2) + (((no_of_drones_per_edge-i+1)*10) ** 2))
                           for i in range(1, no_of_drones_per_edge+2)]
    edge2_angles = [180 - int(math.degrees(math.atan2((no_of_drones_per_edge-i+1),(no_of_drones_per_edge+1))))
                    for i in range(1, no_of_drones_per_edge+2)]
    edge3_displacements = [math.sqrt(
        (edge_length ** 2)+((i*10) ** 2)) for i in range(1, no_of_drones_per_edge+2)]
    edge3_angles = [180 + int(math.degrees(math.atan2(i,(no_of_drones_per_edge+1))))
                    for i in range(1, no_of_drones_per_edge+2)]
    edge4_displacements = [(no_of_drones_per_edge-i+1)*1.414*(edge_length/no_of_drones_per_edge)
                           for i in range(1, no_of_drones_per_edge+1)]
    edge4_angles = [225] * (no_of_drones_per_edge)

followers_distance_to_followee = list(itertools.chain(
    edge1_displacements, edge2_displacements, edge3_displacements, edge4_displacements))  # In meter
followers_azimuth_to_followee = list(itertools.chain(
    edge1_angles, edge2_angles, edge3_angles, edge4_angles))  # In meter

print("Distance List: ")
print(followers_distance_to_followee)
print("Angle List: ")
print(followers_azimuth_to_followee)