#include "tracks_pack_collector.h"

#include "vlog_pretty.h"
#include <algorithm>
#include <assert.h>

//=======================================================================================
Tracks_Pack_Collector::Tracks_Pack_Collector( zcm::ZCM *zcm, Config::ZcmReceive conf )
    : _jpegFrame   ( zcm, conf.jpeg_channel  )
    , _clustersList  ( zcm, conf.lidar_cluster_channel )
    , _max_slippage     ( conf.max_slippage_us           )
    , _need_trace       ( conf.need_trace                )
{
    assert( _max_slippage.count() >= 0 );

    _pack.jpegFrame.service.u_timestamp   = 0;
    _pack.clustersList.service.u_timestamp  = 0;

    _jpegFrame.received.connect( [this](const ZCM_BaslerJpeg& jpegFrame)
    {
        _pack.jpegFrame = jpegFrame;
        _check_emit();

        if (_need_trace)
        {
            VTimePoint tp( std::chrono::microseconds(jpegFrame.service.u_timestamp) );
            vtrace( "Received jpegFrame scan with ts =", tp );
        }
    });
    _clustersList.received.connect( [this](const ZCM_LidarClustersList& clustersList)
    {
        _pack.clustersList = clustersList;
        //_check_emit();
        if (_need_trace)
        {
            VTimePoint tp( std::chrono::microseconds(clustersList.service.u_timestamp) );
            vtrace( "Received clustersList scan with ts =", tp );
        }
    });
}
//=======================================================================================
void Tracks_Pack_Collector::_check_emit()
{
    auto minmax = std::minmax( {_pack.jpegFrame.service.u_timestamp,
                               _pack.clustersList.service.u_timestamp} );

    auto delta = std::chrono::microseconds( minmax.second - minmax.first );
    if ( delta > _max_slippage ) return;

    _pack.processing_begin = VTimePoint::now();
    received( _pack );
}
//=======================================================================================
