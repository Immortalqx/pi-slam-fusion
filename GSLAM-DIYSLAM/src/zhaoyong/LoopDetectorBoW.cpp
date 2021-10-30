#include <stdint.h>
#include <list>

#include <LoopDetector.h>


class LoopDetectorBow : public LoopDetector
{
public:
    LoopDetectorBow(){
        mvInvertedFile.resize(1e5);
    }

    virtual std::string type()const{return "LoopDetectorBow";}

    virtual bool insertMapFrame(const GSLAM::FramePtr& fr)
    {
        GSLAM::BowVector vec;
        if(!fr->getBoWVector(vec)) return false;
        GSLAM::WriteMutex lock(mMutex);
        for(GSLAM::BowVector::const_iterator vit= vec.begin(), vend=vec.end(); vit!=vend; vit++)
            mvInvertedFile[vit->first].push_back(fr->id());

        return true;
    }

    virtual bool eraseMapFrame(const GSLAM::FrameID& frameId){
        return false;
//        GSLAM::BowVector vec;
//        if(!fr->getBoWVector(vec)) return false;
//        // Erase elements in the Inverse File for the entry
//        for(GSLAM::BowVector::const_iterator vit=vec.begin(), vend=vec.end(); vit!=vend; vit++)
//        {
//            // List of keyframes that share the word
//            list<FrameID> &lKFs =   mvInvertedFile[vit->first];

//            for(list<FrameID>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
//            {
//                if(pKF==*lit)
//                {
//                    lKFs.erase(lit);
//                    break;
//                }
//            }
//        }
        return false;
    }

    virtual bool obtainCandidates(const GSLAM::FramePtr& cur,
                                  GSLAM::LoopCandidates& candidates)
    {
        GSLAM::BowVector vec;
        if(!cur->getBoWVector(vec)) return false;
        std::map<GSLAM::FrameID,uint32_t> lKFsSharingWords;

        // Search all keyframes that share a word with current frame

        uint32_t maxCommonWords=1;

        {
            GSLAM::ReadMutex lock(mMutex);

            for(GSLAM::BowVector::const_iterator vit=vec.begin(), vend=vec.end(); vit != vend; vit++)
            {
                std::list<GSLAM::FrameID> &lKFs =   mvInvertedFile[vit->first];

                for(std::list<GSLAM::FrameID>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
                {
                    GSLAM::FrameID pKFi=*lit;
                    if(cur->getParent(pKFi)) continue;
                    std::map<GSLAM::FrameID,uint32_t>::iterator it=lKFsSharingWords.find(pKFi);
                    if(it==lKFsSharingWords.end()) lKFsSharingWords[pKFi]=1;
                    else
                    {
                        uint32_t& count=it->second;
                        count++;
                        if(count>maxCommonWords) maxCommonWords=count;
                    }
                }
            }
        }

        candidates.reserve(lKFsSharingWords.size());
        for(std::pair<GSLAM::FrameID,uint32_t> it:lKFsSharingWords)
            candidates.push_back(GSLAM::LoopCandidate(it.first,1./it.second));

        std::sort(candidates.begin(),candidates.end());
        return candidates.size();
    }

    // Inverted file
    std::vector<std::list<GSLAM::FrameID> >   mvInvertedFile;
    // Mutex
    GSLAM::Mutex                     mMutex;
};

REGISTER_LOOPDETECTOR(LoopDetectorBow,BoW);
