
import os
from os import path
# from typing import ByteString, Iterable

#
import numpy as np
import pandas as pd
# from pandas.core.arrays import string_

import requests
import json


import datetime
from datetime import datetime, timedelta 

import cognite
from cognite.client import CogniteClient
from cognite import data_classes
from cognite.data_classes import FileMetadata




# c = CogniteClient(api_key=os.getenv('COGNITE_API_KEY_ROBOTICS'), 
#                         project="robotics-playground", client_name="Ingvar_Home")
c = CogniteClient(api_key= 'NzE1M2IwMjUtY2VhYS00NmM4LTk1MmUtMjJmMmZjYWUyNjI2', project="robotics-playground", client_name="Ingvar_Home")
NzE1M2IwMjUtY2VhYS00NmM4LTk1MmUtMjJmMmZjYWUyNjI2

## ASSETS ##

# ls = c.assets.list()
# print(ls[0].id)


## FILES ##
# p = b'C:/Users/auest/Documents/Newfolder/test4.txt'
# p2 = 'C:/Users/auest/Documents/Newfolder/test4.txt'

# res = c.files.retrieve(id = 8504219544770468)

# with open('./test6.txt','w') as f:
#    res = f.read()
# print(res)

# file = './SIM_BAG_1.bag'
# file ='C:/Users/Ingvar/Documents/Newfolder/test4.txt'
file = '/home/ingvar2/2021-02-16-12-08-32.bag'
print(file)
hei = open(file, 'r')
print(hei)
h = hei.read()
print(h)
hei.close()

# res = c.files.upload(file,

# tab = pd.read_csv('odom.csv')
# print(tab)

# print(tab['rosbagTimestamp'])
# print(tab['x.3'])

# time = []
# val = []
# ts_id = 7603000938384041

# data = tab['x']
# time = tab['rosbagTimestamp']
# time = time[:-3]
# print(time)
# tm = []

# ts = datetime.fromtimestamp(time).strftime('%Y-%m-%d %H:%M:%S:%f') # 10 siffer

# 1284286794
# 1634490000 000

# print(ts)

# for values in time:
#     print(values)
#     print(int(values))
#     hei = datetime.datetime.fromtimestamp(values)
#     print(hei)

# print(tm)
# ts_epoch = 1362301382


# df = pd.DataFrame({ts_id: data.values}, index = time.values)
# print(df)


# 1634491634480 0000 000     NaN  5790    NaN  1634  480 00 0000 


# start = datetime(2021, 1, 1)
# x = pd.DatetimeIndex([start + timedelta(days=d) for d in range(100)])
# print(x)
# y = np.random.normal(0, 1, 100)
# print(y)
# df = pd.DataFrame({ts_id: y}, index=x)
# print(df)

# res = c.files.upload(file, name='bagfile from sim', source='Ingvar', asset_ids=[4458655741152877])

# print(res[0])
# print('')
# print(res[1])



# res = c.files.upload(path="C:/Users/auest/Documents/Newfolder/test4.txt", 
#                     name="test4", 
#                     # directory="C:/Users/auest/Documents/Newfolder/",
#                     source="Test data",
#                     asset_ids=[4230992780126119])

# class CogniteResource:
#     cognite.c.data_classes._base.CogniteResource

# res = c.files._upload_file_from_path()


# file_metadata = FileMetadata(name="MyFile")
# res = c.files.create(file_metadata)
# url = res[1]

# filepath
# p = b'C:/Users/Ingvar/Documents/Newfolder/test4.txt'
# p2 = 'C:/Users/Ingvar/Documents/Newfolder/test4.txt'

# # URI from metadata upload
# url = 'https://api.cognitedata.com/api/v1/files/gcs_proxy/.....'

# # curl request
# # curl -i -X PUT --data-binary @OBJECT_LOCATION \ -H "Content-Length: OBJECT_SIZE" \ "SESSION_URI"
# size = os.path.getsize(p)
# headers = {'Content-Length' : "{}".format(size)} 
# print(headers)
# data = {'Data-Binary' : p2}
# r = requests.put(url, headers=headers, data=data)



# url = 'https://api.cognitedata.com/api/v1/files/gcs_proxy/upload/storage/v1/b/cognite-storage/o?uploadType=resumable&name=6071055157346185/467415232560871/MyFile&upload_id=ABg5-UwxH9rcCm-VqTQsSfOhURRgU4URX7jqLJFLEDnIytJBqZcAWYc1Rmy-VoL446_tRxZoM5erOXIDg8RoLH6WnBg'

# print(url)
# print(size)

# curl -i -X PUT --data-binary @OBJECT_LOCATION \ -H "Content-Length: OBJECT_SIZE" \ "SESSION_URI"

# # curl -v -H "Content-type: application/json" -X PUT -d '{"latitude":47.629355,"longitude":-122.3794778}' "http://myserver.com/location/private/location"


# res = c.files.upload_bytes(p, name='test byte upload 2 py')

# print(res)



# FilesAPI.upload(asset_ids= external_id=)
# upload_queue = TimeSeriesUploadQueue(cdf_client=c, max_upload_interval=1)

# upload_queue.start()

# while not stop():
#     timestamp, value = source.query()

#     upload_queue.add_to_upload_queue((timestamp, value), external_id="my-timeseries")

# upload_queue.stop()


