# Biến cho các thư viện cần thiết
CXX = g++
CXXFLAGS = -Wall -std=c++17 `pkg-config --cflags opencv4` -Iinclude -I/usr/include/libserial
LDFLAGS = `pkg-config --libs opencv4` -L/usr/lib/x86_64-linux-gnu -lserial   # Đảm bảo thư viện libserial có thể được tìm thấy

# Tên file thực thi
TARGET = lane_detection

# Các file nguồn
SRC = src/main.cpp src/utils.cpp

# Các file object
OBJ = $(SRC:.cpp=.o)

# Biên dịch chương trình
$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LDFLAGS)

# Biên dịch từng file .cpp thành file .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Xoá các file object và file thực thi
clean:
	rm -f $(OBJ) $(TARGET)

# Tái tạo lại tất cả từ đầu
rebuild: clean $(TARGET)
