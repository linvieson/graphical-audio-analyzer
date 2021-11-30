CC = gcc
CFLAGS = -lm

OBJ_D = ./obj
BIN_D = ./bin
SRC_D = ./src
DATA_D = ./data

SOURCE_1 = fft
SOURCE_2 = operate_data
PRG = example


all: makedirs $(BIN_D)/$(PRG)

makedirs:
	mkdir -p $(OBJ_D) $(BIN_D)

$(BIN_D)/$(PRG): $(OBJ_D)/$(SOURCE_1).o $(OBJ_D)/$(SOURCE_2).o $(OBJ_D)/$(PRG).o
	$(CC) $^ -o $@ $(CFLAGS)

$(OBJ_D)/$(SOURCE_1).o: $(SRC_D)/$(SOURCE_1).c
	$(CC) -c $^ -o $@

$(OBJ_D)/$(SOURCE_2).o: $(SRC_D)/$(SOURCE_2).c
	$(CC) -c $^ -o $@

$(OBJ_D)/$(PRG).o: $(SRC_D)/$(PRG).c
	$(CC) -c $^ -o $@


.PHONY : clean
clean:
	rm -rf $(OBJ_D) $(BIN_D)
