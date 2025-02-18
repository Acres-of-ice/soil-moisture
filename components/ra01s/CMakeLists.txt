#set(component_srcs "ra01s.c")

idf_component_register(SRCS "ra01s.c"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "include")


                       