/**
* atomic_signal.h
* Atomic signal broadcaster and listener
* 
* Created by Tony Huang (tony@slamtec.com) at 2020-12-03
* Copyright 2020 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <list>
#include <boost/atomic.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace rpos { namespace system { namespace util {

    class AtomicSignalBroadcaster;
    class AtomicSignalListener;

    class AtomicSignalBroadcasterListenerBase
    {
    protected:
        typedef boost::mutex                    lock_t;
        typedef boost::lock_guard<lock_t>       lock_guard_t;
        typedef boost::atomic_bool              atomic_bool_t;

        enum ListenerDataStatus
        {
            ListenerDataStatusIdle
            , ListenerDataStatusRegistered
            , ListenerDataStatusRemovalRequested
            , ListenerDataStatusRemoved
        };

        struct ListenerData
        {
            atomic_bool_t abIsSignalled;
            boost::atomic<ListenerDataStatus> aeDataStatus;

            explicit ListenerData(bool bIsSignalled);
        };
        typedef boost::shared_ptr<ListenerData>     ListenerData_SharedPtr;
    };

    class AtomicSignalBroadcaster
        : public AtomicSignalBroadcasterListenerBase
        , private boost::noncopyable
    {
    public:
        AtomicSignalBroadcaster();
        ~AtomicSignalBroadcaster();

        // returns true if the listener is not registered anywhere AND is added successfully.
        bool addListener(AtomicSignalListener& rListener, bool bInitSignalled = false);

        // broadcast signal to all listeners.
        void broadcastSignal();

        // remove the listeners that are marked needing being removed.
        void checkToRemoveListeners();

    private:
        void doCleanupWithLock_();

    private:
        lock_t listenerDatLock_;
        std::list<ListenerData_SharedPtr> listenerDatList_;
    };

    class AtomicSignalListener
        : public AtomicSignalBroadcasterListenerBase
        , private boost::noncopyable
    {
        friend class AtomicSignalBroadcaster;
    public:
        explicit AtomicSignalListener(bool bInitSignalled = false);
        ~AtomicSignalListener();

        bool isRegistered() const { return ListenerDataStatusRegistered == listenerDat_->aeDataStatus.load(); }

        bool isSignalled() const { return listenerDat_->abIsSignalled.load(); }
        void setToNoSignalled() { listenerDat_->abIsSignalled.store(false); }
        // returns true iff it is signalled; reset to no signalled if it is signalled.
        bool resetIfSignalled()
        {
            if (!listenerDat_->abIsSignalled.load())
                return false;
            // Note: the two operations "load()" and "store()" are not neccessary to be atomic here.
            listenerDat_->abIsSignalled.store(false);
            return true;
        }

        void unregisterAndReinit(bool bInitSignalled = false);

    private:
        const ListenerData_SharedPtr& getListenerData_() const { return listenerDat_; }

    private:
        ListenerData_SharedPtr listenerDat_;
    };

} } }
