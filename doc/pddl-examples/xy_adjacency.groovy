// Usage: groovy xy_adjacency.groovy 2 2
//
// Sample output:
//
// (can-move x1y1 x2y1)
// (can-move x1y1 x1y2)
// (can-move x1y2 x2y2)
// (can-move x1y2 x1y1)
// (can-move x2y1 x1y1)
// (can-move x2y1 x2y2)
// (can-move x2y2 x1y2)
// (can-move x2y2 x2y1)

size_x = args[0].toInteger()
size_y = args[1].toInteger()

for (x in 1..size_x) {
    for(y in 1..size_y) {
        print "x${x}y${y} "
    }
}
println "- Waypoint\n"


for (x in 1..size_x) {
    for(y in 1..size_y) {
        if (x < size_x) println "(can-move x${x}y${y} x${x+1}y${y})"
        if (x > 1) println "(can-move x${x}y${y} x${x-1}y${y})"
        if (y < size_y) println "(can-move x${x}y${y} x${x}y${y+1})"
        if (y > 1) println "(can-move x${x}y${y} x${x}y${y-1})"
    }
}
