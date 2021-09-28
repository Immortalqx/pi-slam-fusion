/**
******************************************************************************
*
* @file       core.cpp
* @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
* @brief      
* @see        The GNU Public License (GPL) Version 3
* @defgroup   OPMapWidget
* @{
* 
*****************************************************************************/

/* 
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by 
* the Free Software Foundation; either version 3 of the License, or 
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful, but 
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
* for more details.
* 
* You should have received a copy of the GNU General Public License along 
* with this program; if not, write to the Free Software Foundation, Inc., 
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#include <sys/time.h>
#include <sys/timeb.h>
#endif


#include "core.h"

#ifdef DEBUG_CORE
qlonglong internals::Core::debugcounter=0;
#endif


using namespace projections;

namespace internals {


#ifdef _WIN32

void tm_sleep(uint32_t t)
{
    Sleep(t);
}

#else

void tm_sleep(uint32_t t)
{
    struct timespec tp;

    tp.tv_sec  = t / 1000;
    tp.tv_nsec = ( t % 1000 ) * 1000000;

    while( nanosleep(&tp, &tp) );
}

#endif


Core::Core() :
    MouseWheelZooming(false),
    currentPosition(0,0),
    currentPositionPixel(0,0),
    LastLocationInBounds(-1,-1),
    sizeOfMapArea(0,0),
    minOfTiles(0,0),
    maxOfTiles(0,0),
    zoom(0),
    isDragging(false),
    TooltipTextPadding(10,10),
    maxzoom(21),
    runningThreads(0),
    started(false)
{
    mousewheelzoomtype=MouseWheelZoomType::MousePositionAndCenter;
    SetProjection(new MercatorProjection());
    this->setAutoDelete(false);
    renderOffset=Point(0,0);
    dragPoint=Point(0,0);
    CanDragMap=true;

    _threadNum = 5;
    _shouldStop = false;
    ProcessLoadTaskCallback.setMaxThreadCount(_threadNum);

    OPMaps::Instance();
}

Core::~Core()
{
    _shouldStop = true;
    ProcessLoadTaskCallback.waitForDone();
}

void Core::sleep(int millsec)
{
    QCoreApplication::processEvents();
    tm_sleep(millsec);
}

void Core::run()
{
    MrunningThreads.lock();
    ++runningThreads;
    MrunningThreads.unlock();

    while( !_shouldStop )
    {
        bool last = false;
        LoadTask task;

        // get a task
        {
            MtileLoadQueue.lock();

            if(tileLoadQueue.count() > 0)
            {
                task = tileLoadQueue.dequeue();
                {
                    last = (tileLoadQueue.count() == 0);
                }
            }

            MtileLoadQueue.unlock();
        }

        if( !task.HasValue() )
        {
            sleep(20);
            continue;
        }


        // load map title
        {
            Tile* m = Matrix.TileAt(task.Pos);
            if( m==0 || m->Overlays.count() == 0 )
            {
                Tile *t = new Tile(task.Zoom, task.Pos);
                QVector<MapType::Types> layers= OPMaps::Instance()->GetAllLayersOfType(GetMapType());

                foreach(MapType::Types tl, layers)
                {
                    int retry = 0;
                    do
                    {
                        QByteArray img;

                        if( _shouldStop ) break;

                        // tile number inversion(BottomLeft -> TopLeft) for pergo maps
                        if(tl == MapType::PergoTurkeyMap)
                        {
                            img = OPMaps::Instance()->GetImageFrom(tl, Point(task.Pos.X(), maxOfTiles.Height() - task.Pos.Y()), task.Zoom);
                        }
                        else // ok
                        {
                            img = OPMaps::Instance()->GetImageFrom(tl, task.Pos, task.Zoom);
                        }

                        if( _shouldStop ) break;

                        if( img.length()!=0 )
                        {
                            t->Overlays.append(img);
                            break;
                        }
                        else if( OPMaps::Instance()->RetryLoadTile > 0 )
                        {
                            sleep(20);
                        }
                    }
                    while(++retry < OPMaps::Instance()->RetryLoadTile);
                }

                if( t->Overlays.count() > 0 )
                {
                    Matrix.SetTileAt(task.Pos, t);
                    emit OnNeedInvalidation();
                }
                else
                {
                    delete t;
                    t = NULL;
                    emit OnNeedInvalidation();
                }
            }

            if( _shouldStop ) break;

            // last buddy cleans stuff ;}
            if( last )
            {
                OPMaps::Instance()->kiberCacheLock.lockForWrite();
                OPMaps::Instance()->TilesInMemory.RemoveMemoryOverload();
                OPMaps::Instance()->kiberCacheLock.unlock();

                MtileDrawingList.lock();
                Matrix.ClearPointsNotIn(tileDrawingList);
                MtileDrawingList.unlock();

                emit OnTileLoadComplete();
                emit OnNeedInvalidation();
            }

            MtileLoadQueue.lock();
            int nt = tileLoadQueue.size();
            MtileLoadQueue.unlock();
            emit OnTilesStillToLoad(nt<0 ? 0:nt);
        }
    }

    MrunningThreads.lock();
    --runningThreads;
    MrunningThreads.unlock();
}

diagnostics Core::GetDiagnostics()
{
    MrunningThreads.lock();
    diag = OPMaps::Instance()->GetDiagnostics();
    diag.runningThreads=runningThreads;
    MrunningThreads.unlock();
    return diag;
}

void Core::SetZoom(const int &value)
{
    if (!isDragging)
    {
        zoom=value;
        minOfTiles=Projection()->GetTileMatrixMinXY(value);
        maxOfTiles=Projection()->GetTileMatrixMaxXY(value);
        currentPositionPixel=Projection()->FromLatLngToPixel(currentPosition,value);

        if(started)
        {
            MtileLoadQueue.lock();
            tileLoadQueue.clear();
            MtileLoadQueue.unlock();

            Matrix.Clear();

            GoToCurrentPositionOnZoom();
            UpdateBounds();

            emit OnMapDrag();
            emit OnMapZoomChanged();
            emit OnNeedInvalidation();
        }
    }
}

void Core::SetCurrentPosition(const PointLatLng &value)
{
    if(!IsDragging())
    {
        currentPosition = value;
        SetCurrentPositionGPixel(Projection()->FromLatLngToPixel(value, Zoom()));

        if(started)
        {
            GoToCurrentPosition();
            emit OnCurrentPositionChanged(currentPosition);
        }
    }
    else
    {
        currentPosition = value;
        SetCurrentPositionGPixel(Projection()->FromLatLngToPixel(value, Zoom()));

        if(started)
        {
            emit OnCurrentPositionChanged(currentPosition);
        }
    }
}

void Core::SetMapType(const MapType::Types &value)
{

    if(value != GetMapType())
    {
        mapType = value;

        switch(value)
        {

        case MapType::GaoDeMap:
        case MapType::GaoDeSatellite:
        case MapType::GoogleHybridChina:
        case MapType::GoogleMapChina:
        case MapType::GoogleSatelliteChina:
        {
            if(Projection()->Type()!="GCJ02Projection")
            {
                SetProjection(new GCJ02Projection());
                maxzoom=21;
            }
        }
            break;


        case MapType::ArcGIS_Map:
        case MapType::ArcGIS_Satellite:
        case MapType::ArcGIS_ShadedRelief:
        case MapType::ArcGIS_Terrain:
        {
            if(Projection()->Type()!="PlateCarreeProjection")
            {
                SetProjection(new PlateCarreeProjection());
                maxzoom=13;
            }
        }
            break;

        case MapType::ArcGIS_MapsLT_Map_Hybrid:
        case MapType::ArcGIS_MapsLT_Map_Labels:
        case MapType::ArcGIS_MapsLT_Map:
        case MapType::ArcGIS_MapsLT_OrtoFoto:
        {
            if(Projection()->Type()!="LKS94Projection")
            {
                SetProjection(new LKS94Projection());
                maxzoom=11;
            }
        }
            break;

        case MapType::PergoTurkeyMap:
        {
            if(Projection()->Type()!="PlateCarreeProjectionPergo")
            {
                SetProjection(new PlateCarreeProjectionPergo());
                maxzoom=17;
            }
        }
            break;

        case MapType::YandexMapRu:
        {
            if(Projection()->Type()!="MercatorProjectionYandex")
            {
                SetProjection(new MercatorProjectionYandex());
                maxzoom=13;
            }
        }
            break;

        default:
        {
            if(Projection()->Type()!="MercatorProjection")
            {
                SetProjection(new MercatorProjection());
                maxzoom=21;
            }
        }
            break;
        }

        minOfTiles = Projection()->GetTileMatrixMinXY(Zoom());
        maxOfTiles = Projection()->GetTileMatrixMaxXY(Zoom());
        SetCurrentPositionGPixel(Projection()->FromLatLngToPixel(CurrentPosition(), Zoom()));

        if(started)
        {
            // FIXME: CancelAsyncTasks will freeze several seconds
            CancelAsyncTasks();
            OnMapSizeChanged(Width, Height);
            GoToCurrentPosition();
            ReloadMap();
            GoToCurrentPosition();

            emit OnMapTypeChanged(value);
        }
    }
}

void Core::StartSystem()
{
    if(!started)
    {
        // start title download thread
        for(int i=0; i<_threadNum; i++)
            ProcessLoadTaskCallback.start(this);

        started = true;

        ReloadMap();
        GoToCurrentPosition();
    }
}

void Core::UpdateCenterTileXYLocation()
{
    PointLatLng center = FromLocalToLatLng(Width/2, Height/2);
    Point centerPixel = Projection()->FromLatLngToPixel(center, Zoom());
    centerTileXYLocation = Projection()->FromPixelToTileXY(centerPixel);
}

void Core::OnMapSizeChanged(int const& width, int const& height)
{
    Width = width;
    Height = height;

    sizeOfMapArea.SetWidth(1 + (Width/Projection()->TileSize().Width())/2);
    sizeOfMapArea.SetHeight(1 + (Height/Projection()->TileSize().Height())/2);

    UpdateCenterTileXYLocation();

    if(started)
    {
        UpdateBounds();

        emit OnCurrentPositionChanged(currentPosition);
    }
}

void Core::OnMapClose()
{
    _shouldStop = true;
    CancelAsyncTasks();
}

GeoCoderStatusCode::Types Core::SetCurrentPositionByKeywords(QString const& keys)
{
    GeoCoderStatusCode::Types status = GeoCoderStatusCode::Unknow;
    PointLatLng pos = OPMaps::Instance()->GetLatLngFromGeodecoder(keys, status);
    if(!pos.IsEmpty() && (status == GeoCoderStatusCode::G_GEO_SUCCESS))
    {
        SetCurrentPosition(pos);
    }

    return status;
}

RectLatLng Core::CurrentViewArea()
{
    PointLatLng p = Projection()->FromPixelToLatLng(-renderOffset.X(), -renderOffset.Y(), Zoom());
    double rlng = Projection()->FromPixelToLatLng(-renderOffset.X() + Width, -renderOffset.Y(), Zoom()).Lng();
    double blat = Projection()->FromPixelToLatLng(-renderOffset.X(), -renderOffset.Y() + Height, Zoom()).Lat();
    return RectLatLng::FromLTRB(p.Lng(), p.Lat(), rlng, blat);

}

PointLatLng Core::FromLocalToLatLng(int const& x, int const& y)
{
    return Projection()->FromPixelToLatLng(Point(x - renderOffset.X(), y - renderOffset.Y()), Zoom());
}


Point Core::FromLatLngToLocal(PointLatLng const& latlng)
{
    Point pLocal = Projection()->FromLatLngToPixel(latlng, Zoom());
    pLocal.Offset(renderOffset);
    return pLocal;
}

int Core::GetMaxZoomToFitRect(RectLatLng const& rect)
{
    int zoom = 0;

    for(int i = 1; i <= MaxZoom(); i++)
    {
        Point p1 = Projection()->FromLatLngToPixel(rect.LocationTopLeft(), i);
        Point p2 = Projection()->FromLatLngToPixel(rect.Bottom(), rect.Right(), i);

        if(((p2.X() - p1.X()) <= Width+10) && (p2.Y() - p1.Y()) <= Height+10)
        {
            zoom = i;
        }
        else
        {
            break;
        }
    }

    return zoom;
}

void Core::BeginDrag(Point const& pt)
{
    dragPoint.SetX(pt.X() - renderOffset.X());
    dragPoint.SetY(pt.Y() - renderOffset.Y());
    isDragging = true;
}

void Core::EndDrag()
{
    isDragging = false;
    emit OnNeedInvalidation();

}

void Core::ReloadMap()
{
    if(started && !isDragging)
    {
        MtileLoadQueue.lock();
        tileLoadQueue.clear();
        MtileLoadQueue.unlock();

        Matrix.Clear();

        emit OnNeedInvalidation();
    }
}

void Core::GoToCurrentPosition()
{
    // reset stuff
    renderOffset = Point::Empty;
    centerTileXYLocationLast = Point::Empty;
    dragPoint = Point::Empty;

    // goto location
    Drag(Point(-(GetcurrentPositionGPixel().X() - Width/2), -(GetcurrentPositionGPixel().Y() - Height/2)));
}

void Core::GoToCurrentPositionOnZoom()
{
    // reset stuff
    renderOffset = Point::Empty;
    centerTileXYLocationLast = Point::Empty;
    dragPoint = Point::Empty;

    // goto location and centering
    if(MouseWheelZooming)
    {
        if(mousewheelzoomtype != MouseWheelZoomType::MousePositionWithoutCenter)
        {
            Point pt = Point(-(GetcurrentPositionGPixel().X() - Width/2), -(GetcurrentPositionGPixel().Y() - Height/2));
            renderOffset.SetX(pt.X() - dragPoint.X());
            renderOffset.SetY(pt.Y() - dragPoint.Y());
        }
        else // without centering
        {
            renderOffset.SetX(-GetcurrentPositionGPixel().X() - dragPoint.X());
            renderOffset.SetY(-GetcurrentPositionGPixel().Y() - dragPoint.Y());
            renderOffset.Offset(mouseLastZoom);
        }
    }
    else // use current map center
    {
        mouseLastZoom = Point::Empty;

        Point pt = Point(-(GetcurrentPositionGPixel().X() - Width/2), -(GetcurrentPositionGPixel().Y() - Height/2));
        renderOffset.SetX(pt.X() - dragPoint.X());
        renderOffset.SetY(pt.Y() - dragPoint.Y());
    }

    UpdateCenterTileXYLocation();
}

void Core::DragOffset(Point const& offset)
{
    renderOffset.Offset(offset);

    UpdateCenterTileXYLocation();

    if(centerTileXYLocation != centerTileXYLocationLast)
    {
        centerTileXYLocationLast = centerTileXYLocation;
        UpdateBounds();
    }

    {
        LastLocationInBounds = CurrentPosition();
        SetCurrentPosition (FromLocalToLatLng((int) Width/2, (int) Height/2));
    }

    emit OnNeedInvalidation();
    emit OnMapDrag();
}

void Core::Drag(Point const& pt)
{
    renderOffset.SetX(pt.X() - dragPoint.X());
    renderOffset.SetY(pt.Y() - dragPoint.Y());

    UpdateCenterTileXYLocation();

    if(centerTileXYLocation != centerTileXYLocationLast)
    {
        centerTileXYLocationLast = centerTileXYLocation;
        UpdateBounds();
    }

    if(IsDragging())
    {
        LastLocationInBounds = CurrentPosition();
        SetCurrentPosition(FromLocalToLatLng((int) Width/2, (int) Height/2));
    }

    emit OnNeedInvalidation();
    emit OnMapDrag();
}

void Core::CancelAsyncTasks()
{
    if(started)
    {
        MtileLoadQueue.lock();
        tileLoadQueue.clear();
        MtileLoadQueue.unlock();
    }
}

void Core::UpdateBounds()
{
    MtileDrawingList.lock();
    {
        FindTilesAround(tileDrawingList);

        emit OnTileLoadStart();

        MtileLoadQueue.lock();
        foreach(Point p, tileDrawingList)
        {
            LoadTask task = LoadTask(p, Zoom());
            if( !tileLoadQueue.contains(task) )
            {
                tileLoadQueue.enqueue(task);
            }
        }
        MtileLoadQueue.unlock();
    }
    MtileDrawingList.unlock();

    UpdateGroundResolution();
}

void Core::FindTilesAround(QList<Point> &list)
{
    list.clear();;
    for(int i = -sizeOfMapArea.Width(); i <= sizeOfMapArea.Width(); i++)
    {
        for(int j = -sizeOfMapArea.Height(); j <= sizeOfMapArea.Height(); j++)
        {
            Point p = centerTileXYLocation;
            p.SetX(p.X() + i);
            p.SetY(p.Y() + j);

            //if(p.X < minOfTiles.Width)
            //{
            //   p.X += (maxOfTiles.Width + 1);
            //}

            //if(p.X > maxOfTiles.Width)
            //{
            //   p.X -= (maxOfTiles.Width + 1);
            //}

            if(p.X() >= minOfTiles.Width() && p.Y() >= minOfTiles.Height() && p.X() <= maxOfTiles.Width() && p.Y() <= maxOfTiles.Height())
            {
                if(!list.contains(p))
                {
                    list.append(p);
                }
            }
        }
    }
}

void Core::UpdateGroundResolution()
{
    double rez = Projection()->GetGroundResolution(Zoom(), CurrentPosition().Lat());
    pxRes100m =   (int) (100.0 / rez); // 100 meters
    pxRes1000m =  (int) (1000.0 / rez); // 1km
    pxRes10km =   (int) (10000.0 / rez); // 10km
    pxRes100km =  (int) (100000.0 / rez); // 100km
    pxRes1000km = (int) (1000000.0 / rez); // 1000km
    pxRes5000km = (int) (5000000.0 / rez); // 5000km
}
}
