import json
import pandas as pd
from pandas import read_csv
import os

import argparse
import geopy.distance


newFile = "waypoint.json"



# parser = argparse.ArgumentParser(description="json spesifics")
# parser.add_argument("source", help="Input source .csv file")
# parser.add_argument("accuracy", help="Input accuracy in metres (default = 1)")
# args = parser.parse_args()

# try:
#     acc = int(args.accuracy)
# except:
#     acc = 1
# source = args.source

acc = 4
source = "_slash_navsat_slash_fix.csv" #fixed source


# CREATE JSON ##
if not os.path.exists(newFile):
    d = {}
    d["items"] = []
    d["items"].append({
        "_id": "5fb4ede76f4f31296483560c",
        "name": "Plaza Lap",
        "inspectionPoints": [],
        "projectId": "officerobotics",
        "locationId": "cognite-head-office-plaza",
        "waypoints": []
    })

    with open(newFile, "w") as outfile:
        json.dump(d, outfile, indent=4)

    print("Succesfully created new file \"{}\"".format(newFile))

## EXTRACT GPS DATA FROM CSV ##
if os.path.exists(source):
    tab = pd.read_csv(source)
    data_gps = ["latitude", "longitude"]

    lat = tab[data_gps[0]]
    lon = tab[data_gps[1]]
    # Add first coordinate
    way = [{"lat": lat[0], "lon": lon[0]}]

    # Add intermediate coordinates, filtered by metres
    dist = 0.0
    for i in range(1, len(tab)):

        co1 = (lat[i-1], lon[i-1])
        co2 = (lat[i], lon[i])
        dist += geopy.distance.distance(co1, co2).meters
        if dist >= acc:
            way.append({"lat": lat[i-1], "lon": lon[i-1]})
            dist = 0.0

    # Add last coordinate
    way.append({"lat": lat[lat.size-1], "lon": lon[lat.size-1]})

    print("Succesfully read GPS coordinates from \"{}\" with {} metre accuracy".format(source, acc))


    

## APPEND TO JSON ##
# function to add to JSON 
def write_json(data, filename=newFile): 
    with open(filename,"w") as f: 
        json.dump(data, f, indent=4) 
    print("Succesfully added GPS coordinates to \"{}\"".format(newFile))
      
with open(newFile) as json_file: 
    data = json.load(json_file) 
      
    temp = data["items"]

    z = temp[0]["waypoints"]
    
    if range(len(z)) == 0:
        z = way
    else:
        z += way

write_json(data)  