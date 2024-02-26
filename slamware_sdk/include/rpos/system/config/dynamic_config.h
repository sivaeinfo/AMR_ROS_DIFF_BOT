#pragma once
#include <string>
#include <boost/make_shared.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>
#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp> 
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/config/config_parser.h>
#include <json/json.h>

namespace rpos { namespace system { namespace config {

    class RPOS_CORE_API DynamicDispatch
    {
    public:
        static boost::shared_ptr<DynamicDispatch> defaultDispatch();
    public:
        DynamicDispatch() {}
        virtual ~DynamicDispatch() {}
    public:
        bool dispatch(const Json::Value& config);
        bool registeHandler(const boost::function<void(const Json::Value&)>& handler);
        void clear();
    protected:
        boost::signals2::signal<void(const Json::Value&)> handlers_;
    };

    template <typename ConfigT>
    class DynamicConfig
    {
    public:
        DynamicConfig(std::string pattern)
            : writed_(false)
        {
            if (!pattern.empty()) {
                boost::algorithm::split(fields_, pattern, boost::is_any_of(":"));
            }
            DynamicDispatch::defaultDispatch()->registeHandler(boost::bind(&DynamicConfig<ConfigT>::dispatch, this, _1));
        }
        virtual ~DynamicConfig() {}
    public:
        inline const ConfigT& operator*() const
        {
            return config();
        }

        inline const ConfigT* operator->() const
        {
            return &config();
        }
    protected:
        virtual bool onDynamicConfigUpdated(const ConfigT& config) { return true; }
        const ConfigT& config() const {
            if (writed_) {
                {
                    boost::shared_lock<boost::shared_mutex> locker(locker_);
                    parsed_.first = parsed_.second;
                }
                writed_ = false;
            }
            return parsed_.first;
        }
    private:
        virtual bool dispatch(const Json::Value& config)
        {
            const Json::Value* result = patterned(config, fields_);
            if (!result || result->isNull()) {
                return false;
            }
            {
                boost::unique_lock<boost::shared_mutex> locker(locker_);
                ConfigT& write = parsed_.second;
                if (!rpos::system::config::parseConfig(*result, write)) {
                    return false;
                }
            }
            writed_ = true;
            return onDynamicConfigUpdated(this->config());
        }
        const Json::Value* patterned(const Json::Value& config, const std::vector<std::string>& fields)
        {
            const Json::Value* result = &config;

            if (config.isNull()) {
                return &config;
            }
            if (fields.empty()) {
                return &config;
            }
            for (auto iter = fields.begin(); iter != fields.end(); iter++) {
                if (!result->isObject()){
                    return nullptr;
                }
                if (!(*result).isMember(*iter)) {
                    return nullptr;
                }
                const Json::Value& current = (*result)[*iter];
                result = &current;
            }
            return result;
        }
    private:
        std::vector<std::string> fields_;
        mutable boost::atomic<bool> writed_;
        mutable boost::shared_mutex locker_;
        mutable std::pair<ConfigT, ConfigT> parsed_;
    };


} } }