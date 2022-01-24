#include "CBase4618.h"

CBase4618::CBase4618()
{
}

CBase4618::~CBase4618()
{
}

void CBase4618::update()
{
}
    
void CBase4618::draw()
{
}

void CBase4618::run()
{
    do {
        update();
        draw();
    } while (keyPress != 'q');
}
