# 컴파일러 설정
CC = gcc
CFLAGS = -Wall -g

# 라이브러리 플래그 (-li2c 추가)
LIBS = -li2c

# 실행 파일 이름
TARGET = main

# 필요한 소스 파일과 헤더 파일
SRCS = main.c platform.c vl53l8cx_api.c
HEADERS = platform.h vl53l8cx_api.h vl53l8cx_buffers.h

# 오브젝트 파일(.o) 리스트
OBJS = $(SRCS:.c=.o)

# 기본 룰: 컴파일 후 실행
all: $(TARGET)

# 실행 파일 생성 규칙
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LIBS)

# 개별 소스 파일을 오브젝트 파일로 변환하는 규칙
%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

# clean 룰: 생성된 파일 삭제
clean:
	rm -f $(OBJS) $(TARGET)

# 프로그램 실행 규칙
run: $(TARGET)
	./$(TARGET)
