

# define VERBOSE to see executed commands
CC:=g++
LL:=g++
TARGET=hyped
SRCS_DIR:=src
OBJS_DIR:=bin
TOP:=$(shell pwd)

include $(SRCS_DIR)/Source.files
OBJS := $(SRCS:.cpp=.o)

SRCS := $(patsubst %,$(SRCS_DIR)/%,$(SRCS))
OBJS := $(patsubst %,$(OBJS_DIR)/%,$(OBJS))

DEP_DIR := $(OBJS_DIR)
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEP_DIR)/$*.d
INC_DIR := -I$(SRCS_DIR)


ifndef VERBOSE
	Verb := @
endif
Echo := $(Verb)echo


all: $(TARGET)

$(TARGET): $(OBJS)
	$(Echo) "Linking executable $>"
	$(Verb) $(LL) -o $@ $(OBJS)


$(OBJS): $(OBJS_DIR)/%.o: $(SRCS_DIR)/%.cpp
	$(Echo) "Compiling $<"
	$(Verb) mkdir -p $(dir $@)
	$(Verb) $(CC) $(DEPFLAGS) -o $@ -c $(INC_DIR) $<


clean:
	$(Verb) rm -rf $(OBJS_DIR)/*
	$(Verb) rm -f $(TARGET)

define echo_var
	@echo $(1) = $($1)
endef

info:
	$(call echo_var,CC)
	$(call echo_var,TOP)
	$(call echo_var,SRCS)
	$(call echo_var,OBJS)

-include $(OBJS:.o=.d)
