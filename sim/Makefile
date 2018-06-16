ASM_CPP = controller/asm/*.cpp
ASM_H = controller/asm/*.hpp
AASM_CPP = controller/aasm/*.cpp
AASM_H = controller/aasm/*.h

CONTROLLER_CPP = controller/*.cpp $(ASM_CPP) $(AASM_CPP)
CONTROLLER_H = controller/*.h $(ASM_H) $(AASM_H)
ROBOT_CPP = robot/*.cpp
ROBOT_H = robot/*.h
SIMULATION_CPP = simulation/*.cpp
SIMULATION_H = simulation/*.h

ALL_FILES = $(CONTROLLER_H) $(CONTROLLER_CPP) $(ROBOT_H) $(ROBOT_CPP) $(SIMULATION_H) $(SIMULATION_CPP)

all: $(ALL_FILES)
	g++ \
		-Wall \
		main.cpp \
		$(CONTROLLER_CPP) \
		$(ROBOT_CPP) \
		$(SIMULATION_CPP) \
		-framework OpenGL \
		-framework GLUT \
		-lm \
		-I /usr/local/include/ode \
		/usr/local/lib/libode.a \
		-o musclesim

clean:
	rm -f musclesim
