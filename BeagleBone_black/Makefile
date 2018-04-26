

# define VERBOSE to see executed commands
CC:=g++
LL:=g++
TARGET=hyped
MAIN=main.cpp
SRCS_DIR:=src
OBJS_DIR:=bin

CFLAGS:=-pthread -std=c++11 -O3 -Wall
LFLAGS:=-lpthread -pthread

UNAME=$(shell uname)
ifneq ($(UNAME),Linux)
	# assume Windows
	UNAME='Windows'
	CFLAGS:=$(CFLAGS) -DWIN
endif
ARCH=$(shell arch)
ifneq (,$(findstring 64,$(ARCH)))
	CFLAGS:=$(CFLAGS) -DARCH_64
else
  CFLAGS:=$(CFLAGS) -DARCH_32
endif


include $(SRCS_DIR)/Source.files
SRCS := $(SRCS) $(MAIN)
OBJS := $(SRCS:.cpp=.o)

SRCS := $(patsubst %,$(SRCS_DIR)/%,$(SRCS))
OBJS := $(patsubst %,$(OBJS_DIR)/%,$(OBJS))

DEP_DIR := $(OBJS_DIR)
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEP_DIR)/$*.d
INC_DIR := -I$(SRCS_DIR)

# run "make VERBOSE=1" to see all commands
ifndef VERBOSE
	Verb := @
endif
Echo := $(Verb)echo



default: $(TARGET) lint

all:
	$(Verb) set -e; \
	for main in $(MAINS); do \
		$(MAKE) MAIN=$${main}.cpp TARGET=$$main NOLINT=1 --silent; \
	done; \
	$(MAKE) lint --no-print-directory

$(TARGET): $(OBJS)
	$(Echo) "Linking executable $(MAIN) into $@"
	$(Verb) $(LL) $(LFLAGS) -o $@ $(OBJS)


$(OBJS): $(OBJS_DIR)/%.o: $(SRCS_DIR)/%.cpp
	$(Echo) "Compiling $<"
	$(Verb) mkdir -p $(dir $@)
	$(Verb) $(CC) $(DEPFLAGS) $(CFLAGS) -o $@ -c $(INC_DIR) $<

lint:
ifndef NOLINT
	$(Verb) python2.7 utils/Lint/presubmit.py
endif

clean: cleanlint
	$(Verb) rm -rf $(OBJS_DIR)/*
	$(Verb) rm -f $(TARGET)
	$(Verb) rm -f $(MAINS)

cleanlint:
	$(Verb) rm -f .cpplint-cache

define echo_var
	@echo $(1) = $($1)
endef

info:
	$(call echo_var,CC)
	$(call echo_var,TOP)
#	$(call echo_var,SRCS)
#	$(call echo_var,OBJS)
#	$(call echo_var,MAINS)
	$(call echo_var,UNAME)
	$(call echo_var,CFLAGS)

-include $(OBJS:.o=.d)
