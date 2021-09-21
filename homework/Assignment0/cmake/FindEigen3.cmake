# 辅助输出信息
message("now using FindEigen3.cmake find eigen3")

# 将 core 文件路径赋值给 EIGEN3_INCLUDE_DIR
FIND_PATH(EIGEN3_INCLUDE_DIR eigen3 /usr/local/include/)
message("eigen3 dir ${EIGEN3_INCLUDE_DIR}")

