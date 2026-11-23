#include "rigid_mount_calibration_io.hpp"

#include <cerrno>
#include <system_error>
#include <unistd.h>

namespace xxu_bringup::calibration_io {

std::string render_verified_mount(const RigidMount & mount, bool confirmed);
std::string render_verified_body(const RigidMount & mount, bool confirmed);

void validate_output_paths(const OutputFiles & files)
{
  std::set<std::filesystem::path> used;
  for (const auto & name : output_paths(files)) {
    if (name.empty()) {continue;}
    const auto path = std::filesystem::weakly_canonical(std::filesystem::absolute(name));
    if (path.filename() == "rigid_mount.pending.yaml" ||
      std::filesystem::path(name).filename() == "rigid_mount.pending.yaml")
    {
      throw std::invalid_argument("export requires a separate file from rigid_mount.pending.yaml");
    }
    if (!used.insert(path).second) {
      throw std::invalid_argument("mount, body and report output files must be distinct");
    }
  }
}

namespace {

class StagedFile
{
public:
  StagedFile(const std::string & name, const std::string & content)
  : name_(name), temp_(name + ".XXXXXX")
  {
    if (std::filesystem::is_directory(std::filesystem::symlink_status(name))) {
      throw std::system_error(EISDIR, std::generic_category(), name);
    }
    const int fd = mkstemp(temp_.data());
    if (fd < 0) {throw std::system_error(errno, std::generic_category(), name);}
    int error = 0;
    for (std::size_t offset = 0; offset < content.size();) {
      const auto count = write(fd, content.data() + offset, content.size() - offset);
      if (count > 0) {
        offset += count;
      } else if (count == 0 || errno != EINTR) {
        error = count == 0 ? EIO : errno;
        break;
      }
    }
    if (close(fd) != 0 && !error) {error = errno;}
    if (error) {
      unlink(temp_.c_str());
      throw std::system_error(error, std::generic_category(), name);
    }
  }

  StagedFile(const StagedFile &) = delete;
  StagedFile & operator=(const StagedFile &) = delete;

  ~StagedFile()
  {
    if (!temp_.empty()) {unlink(temp_.c_str());}
  }

  void publish()
  {
    if (rename(temp_.c_str(), name_.c_str()) != 0) {
      throw std::system_error(errno, std::generic_category(), name_);
    }
    temp_.clear();
  }

private:
  const std::string & name_;
  std::string temp_;
};

}  // namespace

std::vector<std::string> write_outputs(
  const OutputFiles & files, const CalibrationSamples & data, const CalibrationConfig & config,
  const CalibrationResult & result, const std::optional<Segmentation> & segments,
  const std::optional<VerificationResult> & verification,
  const std::optional<RigidMount> & verified_mount)
{
  validate_output_paths(files);
  std::vector<std::string> errors;
  const auto attempt = [&](const auto & action) {
      try {
        action();
      } catch (const std::exception & e) {
        errors.emplace_back(e.what());
      }
    };
  const auto render_mount = [&](bool confirmed) {
      if (verified_mount) {return render_verified_mount(*verified_mount, confirmed);}
      auto mount_result = result;
      mount_result.confirmed = confirmed;
      return render_rigid_mount_yaml(mount_result, config);
    };
  const auto save_report = [&] {
      if (files.report.empty()) {return;}
      attempt([&] {
        auto exported = result;
        exported.confirmed = result.confirmed && errors.empty();
        exported.reasons.insert(exported.reasons.end(), errors.begin(), errors.end());
        auto reported_verification = verification;
        if (reported_verification) {
          auto & reasons = reported_verification->reasons;
          reasons.insert(reasons.end(), errors.begin(), errors.end());
        }
        StagedFile(files.report, render_report_yaml(data, config, exported, segments,
          reported_verification)).publish();
      });
    };
  const auto save_unconfirmed_mount = [&] {
      attempt([&] {StagedFile(files.mount, render_mount(false)).publish();});
    };

  std::optional<StagedFile> mount, backend;
  if (!files.mount.empty()) {
    attempt([&] {mount.emplace(files.mount, render_mount(result.confirmed));});
  }
  if (!files.backend.empty() && result.confirmed) {
    attempt([&] {
      backend.emplace(files.backend, verified_mount ?
        render_verified_body(*verified_mount, true) :
        render_backend_body_yaml(result, config));
    });
  }
  if (!errors.empty()) {
    if (mount) {save_unconfirmed_mount();}
    save_report();
    return errors;
  }

  save_report();
  if (!errors.empty()) {return errors;}
  if (mount) {attempt([&] {mount->publish();});}
  const bool mount_published = mount && errors.empty();
  // The backend reads common.body directly, so publish it only after all other outputs succeed.
  if (backend && errors.empty()) {attempt([&] {backend->publish();});}
  if (!errors.empty()) {
    if (mount_published && result.confirmed) {save_unconfirmed_mount();}
    save_report();
  }
  return errors;
}

}  // namespace xxu_bringup::calibration_io
