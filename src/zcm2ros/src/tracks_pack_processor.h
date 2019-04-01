#ifndef TRACKS_PACK_PROCESSOR_H
#define TRACKS_PACK_PROCESSOR_H

#include "zcm_deploy.h"

class Tracks_Pack_Processor
{
public:
    void on_received( const Tracks_Pack& pack );
};

#endif // TRACKS_PACK_PROCESSOR_H
