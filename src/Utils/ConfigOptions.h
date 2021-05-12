#pragma once

#include <string>
#include <vector>
#include <exception>
#include <algorithm>
#include <optional>
#include <sstream>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <boost/algorithm/string/predicate.hpp>

namespace LpSlam {

enum class OptionType {
    String,
    Integer,
    Double,
    Bool,
    DoubleList,
    Position,
    Orientation,
    PositionList
};

class Option {
public:
    std::string name = "";
    bool isOptional = false;
    OptionType optionType = OptionType::String;
    
    std::string defaultString = "";
    int defaultInt = 0;
    double defaultDouble = 0.0f;
    bool defaultBool = false;
    std::vector<double> defaultDoubleList;
    Position3 defaultPosition;
    Orientation defaultOrientation;
    std::vector<Position3> defaultPositionList;

    template <class TConfigType> 
    TConfigType getDefault() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <>
inline std::string Option::getDefault() const {
    return defaultString;
}

template <>
inline Position3 Option::getDefault() const {
    return defaultPosition;
}

template <>
inline bool Option::getDefault() const {
    return defaultBool;
}

template <>
inline double Option::getDefault() const {
    return defaultDouble;
}

template <>
inline int Option::getDefault() const {
    return defaultInt;
}

template <>
inline Orientation Option::getDefault() const {
    return defaultOrientation;
}

template <>
inline std::vector<double> Option::getDefault() const {
    return std::vector<double>();
}

template <>
inline std::vector<Position3> Option::getDefault() const {
    return std::vector<Position3>();
}

class ConfigOptions {
public:
    ConfigOptions() {
        // give an empty json dictionary as default
        parse("{}");
    }

    ConfigOptions & required(std::string const& name, OptionType type) {
        Option opt;
        opt.name = name;
        opt.optionType = type;
        opt.isOptional = false;

        m_options.push_back(opt);

        return *this;
    }

    ConfigOptions & optional(std::string const& name, double defaultValue) {
        Option opt;
        opt.name = name;
        opt.optionType = OptionType::Double;
        opt.isOptional = true;
        opt.defaultDouble = defaultValue;

        m_options.push_back(opt);

        return *this;
    }

    ConfigOptions & optional(std::string const& name, int defaultValue) {
        Option opt;
        opt.name = name;
        opt.optionType = OptionType::Integer;
        opt.isOptional = true;
        opt.defaultInt = defaultValue;

        m_options.push_back(opt);

        return *this;
    }

    ConfigOptions & optional(std::string const& name, std::string defaultValue) {
        Option opt;
        opt.name = name;
        opt.optionType = OptionType::String;
        opt.isOptional = true;
        opt.defaultString = defaultValue;

        m_options.push_back(opt);

        return *this;
    }

    ConfigOptions & optional(std::string const& name, bool defaultValue) {
        Option opt;
        opt.name = name;
        opt.optionType = OptionType::Bool;
        opt.isOptional = true;
        opt.defaultBool = defaultValue;

        m_options.push_back(opt);

        return *this;
    }

    ConfigOptions & optional(std::string const& name, std::vector<double> defaultValue) {
        Option opt;
        opt.name = name;
        opt.optionType = OptionType::DoubleList;
        opt.isOptional = true;
        opt.defaultDoubleList = defaultValue;

        m_options.push_back(opt);

        return *this;
    }    

    ConfigOptions & optional(std::string const& name, Position3 defaultValue) {
        Option opt;
        opt.name = name;
        opt.optionType = OptionType::Position;
        opt.isOptional = true;
        opt.defaultPosition = defaultValue;

        m_options.push_back(opt);

        return *this;
    }

    ConfigOptions & optional(std::string const& name, Orientation defaultValue) {
        Option opt;
        opt.name = name;
        opt.optionType = OptionType::Orientation;
        opt.isOptional = true;
        opt.defaultOrientation = defaultValue;

        m_options.push_back(opt);

        return *this;
    }

    ConfigOptions & optional(std::string const& name, std::vector<Position3> defaultValue) {
        Option opt;
        opt.name = name;
        opt.optionType = OptionType::PositionList;
        opt.isOptional = true;
        opt.defaultPositionList = defaultValue;

        m_options.push_back(opt);

        return *this;
    }

    std::string getValidOptionsString() const {
        std::stringstream sout;

        for (auto const& option : m_options) {
            sout << " name: " << option.name << " is optional: " << option.isOptional;
        }
        return sout.str();
    }

    void parse(std::string const& json) {
        size_t nonOptional = std::count_if(m_options.begin(), m_options.end(), [](auto & entry) {
            return !entry.isOptional;
        });

        if ((json.length() == 0) && (nonOptional == 0)) {
            // allow for empty configurations, and don't parse
            // if no values are required;
            parse("{}");
            return;
        }

        m_json = std::nullopt;
        try {
            m_json = nlohmann::json::parse(json);
        }
        catch (nlohmann::json::parse_error & err) {
            spdlog::error("Error while parsing json config {0}",
                err.what());
            throw std::invalid_argument("Cannot parse json config: " + std::string(err.what()));
        }

        // check that we know all entries !
        for (auto const& el : m_json->items()) {
            const std::string key = el.key();
            auto loc = std::find_if(m_options.begin(), m_options.end(), 
                [key](auto const& option) { return option.name == key; });
            if (loc == m_options.end()) {
                if (boost::starts_with(key, "_")) {
                    spdlog::debug("Entry with key " + key + " not a valid configuration option but ignored");
                    continue;
                } else {
                    auto validOptions = getValidOptionsString();
                    spdlog::error("Entry with key {0} not a valid configuration option. valid options are: {1}",
                        key, validOptions);

                    throw std::invalid_argument("Entry with key " + key + " not a valid configuration option");
                }
            }

            try {
                // check that the data type is correct !
                if (loc->optionType == OptionType::Bool) {
                    getBool(loc->name);
                }
                else if (loc->optionType == OptionType::Double) {
                    getDouble(loc->name);
                }
                else if (loc->optionType == OptionType::Integer) {
                    getInteger(loc->name);
                }
                else if (loc->optionType == OptionType::Orientation) {
                    getOrientation(loc->name);
                }
                else if (loc->optionType == OptionType::Position) {
                    getPosition(loc->name);
                }
                else if (loc->optionType == OptionType::String) {
                    getString(loc->name);
                }
                else if (loc->optionType == OptionType::DoubleList) {
                    getDoubleList(loc->name);
                }
                else if (loc->optionType == OptionType::PositionList) {
                    getPositionList(loc->name);
                }
            }
            catch (nlohmann::json::type_error & err)
            {
                spdlog::error("Config entry with name {0}: {1}",
                    loc->name, err.what());
                throw std::invalid_argument("Config entry has wrong type: " + std::string(err.what()));

            }
        }

        // check all the required options are there
        for (auto const& opt: m_options) {
            if (!opt.isOptional) {
                if (m_json->find(opt.name) == m_json->end()) {
                    spdlog::error("Required configuration option " + opt.name + " not found");
                    throw std::invalid_argument("Required configuration option " + opt.name + " not found");
                }
            }
        }
    }

    std::string toString() {
        if (m_json) {
            auto jsonPlusOptional = m_json.value();

            // add all the optional values which have not been
            // explicitly configured

            for (auto const& opt : m_options) {
                if (opt.isOptional) {
                    if (!hasOptional(opt.name)) {
                        // not present, output the default
                        if (opt.optionType == OptionType::Bool) {
                            jsonPlusOptional[opt.name] = getBool(opt.name);
                        } else if (opt.optionType == OptionType::Double) {
                            jsonPlusOptional[opt.name] = getDouble(opt.name);
                        } else if (opt.optionType == OptionType::Integer) {
                            jsonPlusOptional[opt.name] = getInteger(opt.name);
                        } else if (opt.optionType == OptionType::Orientation) {
                            jsonPlusOptional[opt.name] =
                                toJson(getOrientation(opt.name));
                        } else if (opt.optionType == OptionType::Position) {
                            jsonPlusOptional[opt.name] = toJson(getPosition(opt.name));
                        } else if (opt.optionType == OptionType::String) {
                            jsonPlusOptional[opt.name] = getString(opt.name);
                        } else if (opt.optionType == OptionType::DoubleList) {
                            jsonPlusOptional[opt.name] = getDoubleList(opt.name);
                        } else if (opt.optionType == OptionType::PositionList) {
                            jsonPlusOptional[opt.name] = toJson(getPositionList(opt.name));
                        }
                    }
                }
            }

            return jsonPlusOptional.dump();
        } else {
            return "{}";
        }
    }

    std::string getString(std::string const& name) {
        return getGeneric<std::string>(name);
    }

    bool getBool(std::string const& name) {
        return getGeneric<bool>(name);
    }

    int getInteger(std::string const& name) {
        return getGeneric<int>(name);
    }

    double getDouble(std::string const& name) {
        return getGeneric<double>(name);
    }

    std::vector<double> getDoubleList(std::string const& name) {

        auto defReturn = checkPresent<std::vector<double>>(name);
        if (defReturn.has_value()) {
            return *defReturn;
        }

        auto json_obj = (*m_json)[name];
        std::vector<double> double_list;
        for (auto double_entry: json_obj){
            double_list.push_back ( double_entry.get<double>());
        }

        return double_list;
    }

    std::vector<Position3> getPositionList(std::string const& name) {

        auto defReturn = checkPresent<std::vector<Position3>>(name);
        if (defReturn.has_value()) {
            return *defReturn;
        }

        auto json_obj = (*m_json)[name];

        std::vector<Position3> pos_list;
        for (auto pos_entry: json_obj){
            pos_list.push_back ( getPositionFromJsonObject(pos_entry));
        }

        return pos_list;
    }

    Position3 getPosition(std::string const& name) {
        //return getGeneric<double>(name);
        auto defReturn = checkPresent<Position3>(name);
        if (defReturn.has_value()) {
            return *defReturn;
        }

        auto json_obj = (*m_json)[name];
        return getPositionFromJsonObject(json_obj);
    }

    Orientation getOrientation(std::string const& name) {
        //return getGeneric<double>(name);
        auto defReturn = checkPresent<Orientation>(name);
        if (defReturn.has_value()) {
            return *defReturn;
        }

        auto json_obj = (*m_json)[name];

        Orientation ivo(Quaternion(json_obj["w"].get<double>(),
            json_obj["x"].get<double>(),
            json_obj["y"].get<double>(),
            json_obj["z"].get<double>()));

        return ivo;
    }

    /*
    Returns true if an optional config option is present
    */
    bool hasOptional(std::string const& name) const {
        return m_json->find(name) != m_json->end();
    }

private:
    Position3 getPositionFromJsonObject(nlohmann::json const& json_obj) {
        Position3 ivp(Vector3(json_obj["x"].get<double>(),
            json_obj["y"].get<double>(),
            json_obj["z"].get<double>()));

        return ivp;
    }

    nlohmann::json toJson(Orientation const& orient) const {
        nlohmann::json out;
        out["x"] = orient.value.x();
        out["y"] = orient.value.y();
        out["z"] = orient.value.z();
        out["w"] = orient.value.w();

        return out;
    }

    nlohmann::json toJson(Position3 const& pos) const {
        nlohmann::json out;
        out["x"] = pos.value.x();
        out["y"] = pos.value.y();
        out["z"] = pos.value.z();

        return out;
    }

    nlohmann::json toJson(std::vector<Position3> const& posList) const {
        nlohmann::json out;
        for (auto const& pos: posList) {
            out.push_back(toJson(pos));
        }

        return out;
    }

    size_t nonOptionalCount() const {
        size_t nonOptional = std::count_if(m_options.begin(), m_options.end(), [](auto & entry) {
            return !entry.isOptional;
        });
        return nonOptional;
    }


    template <class TEntryType>
    std::optional<TEntryType> checkPresent(std::string const& name) {
        if (m_json->find(name) == m_json->end()) {
            // check if this is optional
            for (auto const& opt : m_options) {
                if (opt.name == name) {
                    return opt.getDefault<TEntryType>();
                }
            }

            spdlog::error("Configuration Option " + name + " not found");
            throw std::invalid_argument("Configuration Option " + name + " not found");
        }
        // is present and needs to be read
        return std::nullopt;
    }

    template <class TEntryType>
    TEntryType getGeneric(std::string const& name) {
        auto defReturn = checkPresent<TEntryType>(name);
        if (defReturn.has_value()) {
            return *defReturn;
        }

        auto json_obj = (*m_json)[name];
        return json_obj.get<TEntryType>();
    }

    std::vector<Option> m_options;
    std::optional<nlohmann::json> m_json;
};

}