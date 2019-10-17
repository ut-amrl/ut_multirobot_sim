SET(target maximal_cliques)
ADD_EXECUTABLE(${target}
    maximal_cliques/main.cpp
    maximal_cliques/maximalcliquegenerator.cpp)

SET(target connected_components)
ADD_EXECUTABLE(${target}
    connected_components/main.cpp
    connected_components/connected_components.cpp)

