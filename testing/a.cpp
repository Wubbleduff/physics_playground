
#include <stdio.h>


namespace class_b
{
    void test();
}


int main(int argc, char **argv)
{
    printf("Hello, world!\n");

    class_b::test();

    return 0;
}

