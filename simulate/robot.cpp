#include "robot.h"

std::string getExecutableDir()
{

    constexpr char kPathSep = '/';

    const char *path = "/proc/self/exe";

    std::string realpath = [&]() -> std::string
    {
        std::unique_ptr<char[]> realpath(nullptr);
        std::uint32_t buf_size = 128;
        bool success = false;
        while (!success)
        {
            realpath.reset(new (std::nothrow) char[buf_size]);
            if (!realpath)
            {
                std::cerr << "cannot allocate memory to store executable path\n";
                return "";
            }

            std::size_t written = readlink(path, realpath.get(), buf_size);
            if (written < buf_size)
            {
                realpath.get()[written] = '\0';
                success = true;
            }
            else if (written == -1)
            {
                if (errno == EINVAL)
                    return path;

                std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
                return "";
            }
            else
            {
                buf_size *= 2;
            }
        }
        return realpath.get();
    }();

    if (realpath.empty())
        return "";

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
        if (realpath.c_str()[i] == kPathSep)
            return realpath.substr(0, i);
    }

    return "";
}

void scanPluginLibraries()
{

    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
        std::printf("Built-in plugins:\n");
        for (int i = 0; i < nplugin; ++i)
        {
            std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
    }

    const std::string sep = "/";

    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
        return;
    }

    const std::string plugin_dir = MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
}

void Robot::resetRobot()
{
    mj_resetData(m, d);
    mj_forward(m, d);
}

void Robot::moveCamera(int action, mjtNum reldx, mjtNum reldy)
{
    mjv_moveCamera(m, action, reldx, reldy, &scn, &cam);
}

void Robot::step()
{
    d->ctrl[0] = 1;
    mj_step(m, d);
}