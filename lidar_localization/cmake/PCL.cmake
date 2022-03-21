find_package(PCL  REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
#APPEND　添加新element到list中
#REMOVE_ITEM　从list中删除某个element
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND ALL_TARGETS_LIBRARIES ${PCL_LIBRARIES})
#把库的名字合并到这个变量中去
