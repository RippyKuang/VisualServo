#include "simulate.h"

int main(int argc, const char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <model.xml>" << std::endl;
        return -1;
    }

    Simulate simulate(argv[1]);
    while (1)
    {
    }
    return 0;
}
