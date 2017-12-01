waypoint_dict = {
    "Waypoint_SH"   : "Shelf",
    "Waypoint_SH#0" : "Shelf#0",
    "Waypoint_SH#1" : "Shelf#1",
    "Waypoint_CB"   : "ConveyorBelt",
    "Waypoint_CB#0" : "ConveyorBelt#0",
    "Waypoint_CB#1" : "ConveyorBelt#1",
    "Waypoint_RS"   : "RechargeStation",
    "Waypoint_RS#0" : "RechargeStation#0"
}

target_dict = {
    "Truck#0" : "ConveyorBelt",
    "Truck#1" : "ConveyorBelt#0",
    "Truck#2" : "ConveyorBelt#1"
}

wp_list = ["Waypoint_SH", "Waypoint_SH#0", "Waypoint_SH#1", "Waypoint_CB",
            "Waypoint_CB#0", "Waypoint_CB#1", "Waypoint_RS",
            "Waypoint_RS#0"]

poi_list = ["Shelf", "Shelf#0", "Shelf#1", "ConveyorBelt",
            "ConveyorBelt#0", "ConveyorBelt#1", "RechargeStation",
            "RechargeStation#0"]

poi_pose = {
    'RechargeStation#0': [0.8750001192092896, -3.5351505279541016,
                          1.2519996166229248],
    'ConveyorBelt': [2.9749975204467773, -3.535163164138794,
                     1.2519997358322144],
    'Shelf#1': [0.9129988551139832, -2.0351452827453613,
                1.2520004510879517],
    'Shelf#0': [0.9129999876022339, -0.5351451635360718,
                1.2520002126693726],
    'ConveyorBelt#0': [2.9749979972839355, -0.5351498126983643,
                       1.2519997358322144],
    'ConveyorBelt#1': [2.9749975204467773, 2.464836835861206,
                       1.2519997358322144],
    'RechargeStation': [0.874998927116394, 2.464848041534424,
                        1.2520004510879517],
    'Shelf': [0.9130004644393921, 0.9648550748825073,
              1.252000093460083]
}


cost_matrix = [
    [0, 15.0001, 30.0, 31.7845, 10.5, 24.7497, 18.2016, 12.7494],
    [15.0001, 0, 14.9999, 18.3098, 18.3098, 12.7494, 10.31, 12.7493],
    [30.0, 14.9999, 0, 10.5, 31.7844, 12.7493, 18.2015, 24.7496],
    [31.7845, 18.3098, 10.5, 0, 30.0, 7.5024, 15.0012, 22.5008],
    [10.5, 18.3098, 31.7844, 30.0, 0, 22.5008, 15.0012, 7.5024],
    [24.7497, 12.7494, 12.7493, 7.5024, 22.5008, 0, 7.5, 15.0],
    [18.2016, 10.31, 18.2015, 15.0012, 15.0012, 7.5, 0, 7.5],
    [12.7494, 12.7493, 24.7496, 22.5008, 7.5024, 15.0, 7.5, 0]
]    

pick_avg_time = 24
drop_avg_time = 20
