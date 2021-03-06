#ifndef GSLAM_TILEMANAGER_H
#define GSLAM_TILEMANAGER_H

#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/TileProjection.h>
#include <list>

namespace GSLAM {

class TileBase: public GObject
{
public:
    virtual std::string type()const{return "TileBase";}
    virtual GImage  getTileImage(){return GImage();}
    virtual GImage  getTileDem(){return GImage();}
    virtual GImage getTileWeight() {return GImage(); }
    virtual GImage getTileQuad() {return GImage();}
    virtual void   setTileQuad(GImage& quad) {return;}
    virtual unsigned int getTileTexid() const {return -1;}
    virtual void setTileTexid(unsigned int texid) { return;}

    virtual Point3i getTilePosition(){return Point3i(0,0,-1);}// invalid when z<0, WARNING: NOT ALWAYS VALID!
    virtual double  getTimeStamp(){return -1;}// invalid when <0, WARNING: NOT ALWAYS VALID!
    virtual int     memSizeInBytes()const{return -1;}// invalid when <0
    virtual bool    toStream(std::ostream& ostream){return false;}
    virtual bool    fromStream(std::istream& istream){return false;}

    virtual bool    modified(void) { return false; }
    virtual void    setModified(bool modifed=true) {}

    virtual void    release(void) { }

    static int64_t hashVal(int _x, int _y, int _z)
    {
        int64_t iz = _z, iy = _y, ix = _x;
        int64_t v = iz << 48 | iy << 24 | ix;
        return v;
    }

    template <typename T>
    static void read(std::istream& ist,T& obj){ist.read((char*)&obj,sizeof(obj));}

    template <typename T>
    static void write(std::ostream& ost,T& obj){ost.write((char*)&obj,sizeof(obj));}
};
typedef SPtr<TileBase> TilePtr;

class ImageTile : public TileBase
{
public:
    ImageTile(const GImage& image=GImage()):_image(image), _modified(false) {}

    virtual std::string type()const{return "ImageTile";}
    virtual GImage getTileImage(){return _image;}
    virtual int    memSizeInBytes()const{return _image.total()*_image.elemSize();}// invalid when <0

    virtual bool    modified(void) { return _modified; }
    virtual void    setModified(bool modifed=true) { _modified = modifed; }

    virtual bool   toStream(std::ostream& ostream)
    {
        ostream.write((char*)&_image.cols,sizeof(int)*3);
        if(_image.empty())
            return true;
        ostream.write((char*)_image.data,memSizeInBytes());
        return true;
    }

    virtual bool fromStream(std::istream& istream)
    {
        int cols,rows,flag;
        istream .read((char*)&cols,sizeof(int))
                .read((char*)&rows,sizeof(int))
                .read((char*)&flag,sizeof(int));
        GImage image(cols,rows,flag);
        if(image.empty()) return true;
        istream.read((char*)image.data,image.total()*image.elemSize());
        _image=image;
        _modified = false;
        return true;
    }

    GImage  _image;
    bool    _modified;
};
typedef SPtr<ImageTile> ImageTilePtr;

class TerrainTile : public ImageTile
{
public:
    TerrainTile(const GImage& image=GImage(),const GImage& dem=GImage())
        :ImageTile(image),_dem(dem){}
    virtual std::string type()const{return "TerrainTile";}

    virtual GImage getTileDem() {return _dem;}
    virtual void setTileDem(const GImage& dem) {_dem = dem;}

    virtual int    memSizeInBytes()const{return ImageTile::memSizeInBytes()+_dem.total()*_dem.elemSize();}// invalid when <0
    virtual bool   toStream(std::ostream& ostream)
    {
        if(!ImageTile::toStream(ostream)) return false;
        ostream.write((char*)&_dem.cols,sizeof(int)*3);
        if(_dem.empty()) return true;
        ostream.write((char*)_dem.data,_dem.total()*_dem.elemSize());
        return true;
    }

    virtual bool fromStream(std::istream& istream)
    {
        if(!ImageTile::fromStream(istream)) return false;
        int cols,rows,flag;
        istream .read((char*)&cols,sizeof(int))
                .read((char*)&rows,sizeof(int))
                .read((char*)&flag,sizeof(int));
        GImage dem(cols,rows,flag);
        if(dem.empty()) return true;
        istream.read((char*)dem.data,dem.total()*dem.elemSize());
        _dem=dem;
        _modified = false;
        return true;
    }

    GImage  _dem;
};
typedef SPtr<TerrainTile> TerrainTilePtr;

class TerrainTileWithInfo: public TerrainTile
{
  public:
    TerrainTileWithInfo(const GImage& image=GImage(),const GImage& dem=GImage(),
                        const Point3i& location=Point3i(0,0,-1),const double& time=-1)
        : TerrainTile(image,dem),_location(location),_timestamp(time){}

    virtual Point3i getTilePosition(){return _location;}// invalid when z<0
    virtual double  getTimeStamp(){return _timestamp;}// invalid when <0
    virtual void    setTimeStamp(double time) {time = _timestamp;}

    virtual int     memSizeInBytes()const{return TerrainTile::memSizeInBytes()+sizeof(_location)+sizeof(_timestamp);}// invalid when <0

    virtual bool    toStream(std::ostream& s)
    {
        write(s,_image.cols); write(s,_image.rows); write(s,_image.flags);
        write(s,_dem.cols); write(s,_dem.rows); write(s,_dem.flags);
        write(s,_location); write(s,_timestamp);
        if(!_image.empty()) s.write((char*)_image.data,_image.total()*_image.elemSize());
        if(!_dem.empty()) s.write((char*)_dem.data,_dem.total()*_dem.elemSize());
        return true;
    }

    virtual bool fromStream(std::istream& s)
    {
        read(s,_image.cols); read(s,_image.rows); read(s,_image.flags);
        read(s,_dem.cols); read(s,_dem.rows); read(s,_dem.flags);
        read(s,_location); read(s,_timestamp);
        _image= GImage(_image.rows,_image.cols,_image.flags);
        _dem  = GImage(_dem.rows,_dem.cols,_dem.flags);
        if(!_image.empty()) s.read((char*)_image.data,_image.total()*_image.elemSize());
        if(!_dem.empty()) s.read((char*)_dem.data,_dem.total()*_dem.elemSize());
        _modified = false;

        return true;
    }

    Point3i _location;
    double  _timestamp;
};
typedef SPtr<TerrainTileWithInfo> TerrainTileWithInfoPtr;

class TileManager: public GObject
{
public:
    struct TileArea{
    public:
        TileArea():level(0),min(INT32_MAX,INT32_MAX),max(INT32_MIN,INT32_MIN){}

        Point2i getSize()const{return max-min;}
        int     getLevel()const{return level;}
        Point2i getMin()const{return min;}
        Point2i getMax()const{return max;}

        void    setLevel(int level_){level=level_;}
        void    setMin(const Point2i& min_){min=min_;}
        void    setMax(const Point2i& max_){max=max_;}

        int     level;
        Point2i min,max;
    };

    virtual ~TileManager(){}
    virtual std::string type()const{return "TileManager";}

    virtual TilePtr getTile(int x,int y,int z){return TilePtr();}
    virtual bool    setTile(int x,int y,int z, const TilePtr& tile){return false;}
    virtual std::list<TilePtr> getAllTile() {return std::list<TilePtr>{};}
    virtual int     maxZoomLevel()const{return _area.level;}
    virtual int     minZoomLevel()const{return -1;}
    virtual bool    save(const std::string& file){return false;}

    TileArea          getTileArea()const{return   _area;}
    TileProjectionPtr getProjection()const{return _projection;}

    void setTileArea(TileArea area){_area=area;}
    void setProjection(TileProjectionPtr projection){_projection=projection;}
protected:
    TileArea _area;
    TileProjectionPtr _projection;
};
typedef SPtr<TileManager> TileManagerPtr;

}


#endif

