INC_PATH=-I/usr/local/include/player-3.0

LIB_PATH=-L/usr/local/lib64 -lplayerc -lm -lz -lplayerinterface -lplayerwkb -lplayercommon

move: move3.c
	gcc -o move1 $(INC_PATH) move3.c $(LIB_PATH)

clean:
	rm -rf move1
