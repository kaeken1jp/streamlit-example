import numpy as np
import pandas as pd
from scipy.spatial import distance_matrix

import streamlit as st
from streamlit_folium import folium_static
import folium

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


# マップ及びマップの初期座標、初期ズームの指定
m = folium.Map(location=[35.4122, 137.4130], zoom_start=6)

# 緯度、経度のDataFrameを作成
data = pd.DataFrame(
    [
    ["東京", 35.4122, 139.4130],
    ["千葉", 35.6047, 140.1233],
    ["山梨", 35.6638, 138.5683],
    ["宮城", 38.2688, 140.8719],
    ["新潟", 37.9022, 139.0236],
    ["長野", 36.6513, 138.1811],
    ["福島", 37.7500, 140.4677],
    ["栃木", 36.5658, 139.8836],
    ["石川", 36.5944, 136.6255],
    ["福井", 36.0652, 136.2219],
    ["静岡", 34.9769, 138.3830],
    ["岐阜", 35.3911, 136.7222],
    ["三重", 34.7302, 136.5086],
    ["愛知", 35.1802, 136.9066],
    ["京都", 35.0213, 135.7555],
    ["和歌山", 34.2261,135.1675],
    ["鳥取", 35.5036, 134.2383],
    ["島根", 35.4722, 133.0505],
    ["隠岐の島", 36.1200, 133.1000],
    #["", , ],
    ["広島", 34.3963, 132.4594],
    ["高知", 33.5597, 133.5311],
    ["福岡", 33.6063, 130.4180],
    ["大島", 34.7570, 139.3574],
    ["八丈島", 33.1024, 139.7990],
    ["鹿児島", 31.5602, 130.5580],
    ]
)
data.columns = ["place", "latitude", "longitude"]

### サイドバー

# ヘリの台数
vehicle_num = st.sidebar.selectbox('ヘリの台数',(1, 2, 3, 4))


# 県を選択するマルチラベル（出発点の東京除く）
prefecture = st.sidebar.multiselect('出発点の東京除くマーカーを表示する県を選択してください',list(data[data.place!="東京"].place) , 
                list(data[data.place!="東京"].place)
) #例

# 東京を追加
prefecture = pd.concat(
    [
     data[data.place=="東京"].place,
     data.place[data.place.isin(prefecture)]
    ]
)

data = data[data["place"].isin(prefecture)]

### タイトル
st.title(f"出発点東京 {len(data)}拠点 ヘリ{vehicle_num}機のルート最適化")

# 距離行列
data_matrix = pd.DataFrame(
  distance_matrix(data[["latitude",	"longitude"]].values,
  data[["latitude",	"longitude"]].values),
  index=data["place"],
  columns=data["place"]
)

### 以下、数理最適化
manager = pywrapcp.RoutingIndexManager(
    len(data),                            
    vehicle_num, # 車両台数 
    0  # 出発点のindex
)

routing = pywrapcp.RoutingModel(manager)

def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Convert from routing variable Index to distance matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data_matrix.values.tolist()[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)

routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

dimension_name = 'Distance'
routing.AddDimension(
    transit_callback_index,
    0,  # no slack
    3000,  # vehicle maximum travel distance
    True,  # start cumul to zero
    dimension_name)
distance_dimension = routing.GetDimensionOrDie(dimension_name)
distance_dimension.SetGlobalSpanCostCoefficient(100)

search_parameters = pywrapcp.DefaultRoutingSearchParameters()

search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

solution = routing.SolveWithParameters(search_parameters)

def get_routes(solution, routing, manager):
  """Get vehicle routes from a solution and store them in an array."""
  # Get vehicle routes and store them in a two dimensional array whose
  # i,j entry is the jth location visited by vehicle i along its route.
  routes = []
  for route_nbr in range(routing.vehicles()):
    index = routing.Start(route_nbr)
    route = [manager.IndexToNode(index)]
    while not routing.IsEnd(index):
      index = solution.Value(routing.NextVar(index))
      route.append(manager.IndexToNode(index))
    routes.append(route)
  return routes

routes = get_routes(solution, routing, manager)
### 以上、数理最適化


# 線で結ぶ
sq = [] # 座標を格納するリスト
color_list = [] # 色指定用のリスト

# sq、color_list の入力
for i in range(len(routes)):
    sq = sq + data.iloc[routes[i], 1:3].values.tolist()
    color_list = color_list + [i] * len(routes[i])


# order 経路順の作成
place_index = [0]
order_num = [0]

for i in range(len(routes)):
    for j in range(1, len(routes[i]) -1):
        place_index = place_index + [routes[i][j]]
        order_num = order_num + [j]

order = pd.DataFrame(
    [
    order_num,
    place_index
    ]
).T

order.columns = ["order_num", "place_index"]

order = order.sort_values("place_index").reset_index()["order_num"]

# 経路の作成
folium.ColorLine(       # 色付きの線をマップに表示
    positions=sq,       # 座標
    colors= color_list, # 色
    weight=3            # 線の太さ
).add_to(m)

# マーカーの表示
for i in range(len(data)):

  popup = folium.Popup(
    html= f"{order[i]}",
    max_width=1000,
    show=False
  )

  folium.Circle(
    location = data[["latitude", "longitude"]].values.tolist()[i],
    popup=popup,
    parse_html=True,
    color = "red",
    radius = 10000.0,
    fill = True
    ).add_to(m)

# 地図をブラウザに表示
folium_static(m)

st.write("注)各拠点にヘリが停まれるかどうかは無視")
