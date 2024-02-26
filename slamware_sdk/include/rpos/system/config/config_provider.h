/**
* config_provider.h
* Dependency Injectable Configurations
* 
* Created by Tony Huang (tony@slamtec.com) at 2020-12-03
* Copyright 2020 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/system/util/atomic_signal.h>
#include <rpos/context/base_service.h>

#include <boost/optional.hpp>

namespace rpos { namespace system { namespace config {

    /**
    * denotes a configuration injector point
    *
    * ## Examples
    * ```cpp
    * struct DwaLocalPlannerConfig {
    *   int some_value;
    * };
    * 
    * // in the application
    * applicationContext->registerService<IConfig<DwaLocalPlannerConfig>>(boost::make_shared<Config<DwaLocalPlannerConfig>("motionplan.dwa"));
    *
    * // in action
    * class DwaAction : public Action {
    * private:
    *     ServiceDependency<IConfigProvider<boost::shared_ptr<DwaLocalPlannerConfig>>> config_;
    *
    * public:
    *     DwaAction()
    *         : config_("motionplan.dwa")
    *     {
    *         depends(config_);
    *     }
    *
    *     virtual void onTick(IActionTickContext& ctx) {
    *         boost::shared_ptr<DwaLocalPlannerConfig> config;
    *         if (config_->getConfig(config)) {
    *             // do sth
    *         }
    *     }
    * };
    * ```
    */
    template < class T >
    class IConfigProvider
    {
    public:
        virtual ~IConfigProvider() {}

    public:
        /**
         * @brief Get current config
         * @return `false` if no config set
        */
        virtual bool getConfig(T& refCfg) = 0;

        /**
         * @brief Update config
         * @param cfg Upcoming config
        */
        virtual void updateConfig(T&& cfg) = 0;

        /**
         * @brief Add atomic listener to receive config changed events
         * @param listener The listener
         * @param initSignalled If it should be marked as signalled at initialization (default `false`)
         * @return `true` if listener is not registered elsewhere AND registered successfully
        */
        virtual bool addListener(rpos::system::util::AtomicSignalListener& listener, bool initSignalled = false) = 0;

        /**
         * @brief Check to remove listeners that marked as to be removed
        */
        virtual void checkToRemoveListeners() = 0;
    };

    template < class T >
    class ConfigProviderService
        : public rpos::context::BaseService<ConfigProviderService<T>>
        , public IConfigProvider<T>
    {
    public:
        ConfigProviderService()
        {
            this->template provides<IConfigProvider<T>>();
        }

        explicit ConfigProviderService(const T& cfg)
            : config_(cfg)
        {
            this->template provides<IConfigProvider<T>>();
        }

        virtual ~ConfigProviderService()
        {
        }

        virtual bool getConfig(T& refCfg)
        {
            boost::lock_guard<boost::mutex> guard(this->lock);
            if (config_.has_value())
            {
                refCfg = *config_;
                return true;
            }
            else
            {
                return false;
            }
        }

        virtual void updateConfig(T&& cfg)
        {
            boost::lock_guard<boost::mutex> guard(this->lock);
            config_ = std::move(cfg);
            broadcaster_.broadcastSignal();
        }

        virtual bool addListener(util::AtomicSignalListener& listener, bool initSignalled)
        {
            boost::lock_guard<boost::mutex> guard(this->lock);
            return broadcaster_.addListener(listener, initSignalled);
        }

        void checkToRemoveListeners()
        {
            boost::lock_guard<boost::mutex> guard(this->lock);
            broadcaster_.checkToRemoveListeners();
        }

    protected:
        bool onStart() { return true; }
        bool onStop() { return true; }

    private:
        boost::optional<T> config_;
        rpos::system::util::AtomicSignalBroadcaster broadcaster_;
    };
    
} } }
