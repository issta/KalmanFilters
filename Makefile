
OBJS = kf_test.o kf_attstate.o sd_kf_tbl.o math_entry.o matrix3x3d.o matrix3x4d.o matrix4x3d.o matrix6x6d.o matrixmxnd.o quaternion.o scalar.o vector3d.o vector4d.o vector6d.o

default: $(OBJS) 
	gcc -o kf_test $(OBJS) -lm


clean::
	rm $(OBJS) kf_test


kf_test.o: kf_test.c
	gcc -c -D_ix86_ -I math/fsw/inc kf_test.c

kf_attstate.o: kf_attstate.c
	gcc -c -D_ix86_ -I math/fsw/inc kf_attstate.c

sd_kf_tbl.o: sd_kf_tbl.c
	gcc -c -D_ix86_ -I math/fsw/inc sd_kf_tbl.c

math_entry.o: math/fsw/src/math_entry.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/math_entry.c

matrix3x3d.o: math/fsw/src/matrix3x3d.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/matrix3x3d.c

matrix3x4d.o: math/fsw/src/matrix3x4d.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/matrix3x4d.c

matrix4x3d.o: math/fsw/src/matrix4x3d.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/matrix4x3d.c

matrix6x6d.o: math/fsw/src/matrix6x6d.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/matrix6x6d.c

matrixmxnd.o: math/fsw/src/matrixmxnd.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/matrixmxnd.c

quaternion.o: math/fsw/src/quaternion.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/quaternion.c

scalar.o: math/fsw/src/scalar.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/scalar.c

vector3d.o: math/fsw/src/vector3d.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/vector3d.c

vector4d.o: math/fsw/src/vector4d.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/vector4d.c

vector6d.o: math/fsw/src/vector6d.c
	gcc -c -D_ix86_ -I math/fsw/inc math/fsw/src/vector6d.c

