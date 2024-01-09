#include <iostream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <mutex>
#include <vector>
#include "sqlite3.h"

using namespace std;
std::mutex update_schema_mutex_;
const static int COLMAP_VERSION_NUMBER = 3800;

inline int SQLite3CallHelper(const int result_code, const std::string& filename,
                             const int line_number) {
  switch (result_code) {
    case SQLITE_OK:
    case SQLITE_ROW:
    case SQLITE_DONE:
      return result_code;
    default:
      fprintf(stderr, "SQLite error [%s, line %i]: %s\n", filename.c_str(),
              line_number, sqlite3_errstr(result_code));
      exit(EXIT_FAILURE);
  }
}
#define SQLITE3_CALL(func) SQLite3CallHelper(func, __FILE__, __LINE__)

#define SQLITE3_EXEC(database, sql, callback)                                 \
  {                                                                           \
    char* err_msg = nullptr;                                                  \
    const int result_code =                                                   \
        sqlite3_exec(database, sql, callback, nullptr, &err_msg);             \
    if (result_code != SQLITE_OK) {                                           \
      fprintf(stderr, "SQLite error [%s, line %i]: %s\n", __FILE__, __LINE__, \
              err_msg);                                                       \
      sqlite3_free(err_msg);                                                  \
    }                                                                         \
  }

void StringAppendV(std::string* dst, const char* format, va_list ap) {
  // First try with a small fixed size buffer.
  static const int kFixedBufferSize = 1024;
  char fixed_buffer[kFixedBufferSize];

  // It is possible for methods that use a va_list to invalidate
  // the data in it upon use.  The fix is to make a copy
  // of the structure before using it and use that copy instead.
  va_list backup_ap;
  va_copy(backup_ap, ap);
  int result = vsnprintf(fixed_buffer, kFixedBufferSize, format, backup_ap);
  va_end(backup_ap);

  if (result < kFixedBufferSize) {
    if (result >= 0) {
      // Normal case - everything fits.
      dst->append(fixed_buffer, result);
      return;
    }

#ifdef _MSC_VER
    // Error or MSVC running out of space.  MSVC 8.0 and higher
    // can be asked about space needed with the special idiom below:
    va_copy(backup_ap, ap);
    result = vsnprintf(nullptr, 0, format, backup_ap);
    va_end(backup_ap);
#endif

    if (result < 0) {
      // Just an error.
      return;
    }
  }

  // Increase the buffer size to the size requested by vsnprintf,
  // plus one for the closing \0.
  const int variable_buffer_size = result + 1;
  std::unique_ptr<char> variable_buffer(new char[variable_buffer_size]);

  // Restore the va_list before we use it again.
  va_copy(backup_ap, ap);
  result =
      vsnprintf(variable_buffer.get(), variable_buffer_size, format, backup_ap);
  va_end(backup_ap);

  if (result >= 0 && result < variable_buffer_size) {
    dst->append(variable_buffer.get(), result);
  }
}

std::string StringPrintf(const char* format, ...) {
  va_list ap;
  va_start(ap, format);
  std::string result;
  StringAppendV(&result, format, ap);
  va_end(ap);
  return result;
}

bool ExistsColumn(const std::string& table_name,
                            const std::string& column_name, sqlite3* save_database_){
  const std::string sql =
      StringPrintf("PRAGMA table_info(%s);", table_name.c_str());

  sqlite3_stmt* sql_stmt;
  SQLITE3_CALL(sqlite3_prepare_v2(save_database_, sql.c_str(), -1, &sql_stmt, 0));

  bool exists_column = false;
  while (SQLITE3_CALL(sqlite3_step(sql_stmt)) == SQLITE_ROW) {
    const std::string result =
        reinterpret_cast<const char*>(sqlite3_column_text(sql_stmt, 1));
    if (column_name == result) {
      exists_column = true;
      break;
    }
  }

  SQLITE3_CALL(sqlite3_finalize(sql_stmt));

  return exists_column;
}

void CreateCameraTable(sqlite3** save_database_){
  const std::string sql =
      "CREATE TABLE IF NOT EXISTS cameras"
      "   (camera_id            INTEGER  PRIMARY KEY AUTOINCREMENT  NOT NULL,"
      "    model                INTEGER                             NOT NULL,"
      "    width                INTEGER                             NOT NULL,"
      "    height               INTEGER                             NOT NULL,"
      "    params               BLOB,"
      "    prior_focal_length   INTEGER                             NOT NULL);";

  SQLITE3_EXEC(*save_database_, sql.c_str(), nullptr);
}

void CreateImageTable(sqlite3** save_database_){
  const std::string sql = StringPrintf(
      "CREATE TABLE IF NOT EXISTS images"
      "   (image_id   INTEGER  PRIMARY KEY AUTOINCREMENT  NOT NULL,"
      "    name       TEXT                                NOT NULL UNIQUE,"
      "    camera_id  INTEGER                             NOT NULL,"
      "    prior_qw   REAL,"
      "    prior_qx   REAL,"
      "    prior_qy   REAL,"
      "    prior_qz   REAL,"
      "    prior_tx   REAL,"
      "    prior_ty   REAL,"
      "    prior_tz   REAL,"
      "CONSTRAINT image_id_check CHECK(image_id >= 0 and image_id < %d),"
      "FOREIGN KEY(camera_id) REFERENCES cameras(camera_id));"
      "CREATE UNIQUE INDEX IF NOT EXISTS index_name ON images(name);",
      static_cast<size_t>(std::numeric_limits<int32_t>::max()));

  SQLITE3_EXEC(*save_database_, sql.c_str(), nullptr);
}

void CreateKeypointsTable(sqlite3** save_database_){
  const std::string sql =
      "CREATE TABLE IF NOT EXISTS keypoints"
      "   (image_id  INTEGER  PRIMARY KEY  NOT NULL,"
      "    rows      INTEGER               NOT NULL,"
      "    cols      INTEGER               NOT NULL,"
      "    data      BLOB,"
      "FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE);";

  SQLITE3_EXEC(*save_database_, sql.c_str(), nullptr);
}

void CreateDescriptorsTable(sqlite3** save_database_){

  const std::string sql =
      "CREATE TABLE IF NOT EXISTS descriptors"
      "   (image_id  INTEGER  PRIMARY KEY  NOT NULL,"
      "    rows      INTEGER               NOT NULL,"
      "    cols      INTEGER               NOT NULL,"
      "    data      BLOB,"
      "FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE);";

  SQLITE3_EXEC(*save_database_, sql.c_str(), nullptr);
}

void CreateMatchesTable(sqlite3** save_database_){

  const std::string sql =
      "CREATE TABLE IF NOT EXISTS matches"
      "   (pair_id  INTEGER  PRIMARY KEY  NOT NULL,"
      "    rows     INTEGER               NOT NULL,"
      "    cols     INTEGER               NOT NULL,"
      "    data     BLOB);";

  SQLITE3_EXEC(*save_database_, sql.c_str(), nullptr);
}

void CreateTwoViewGeometriesTable(sqlite3** save_database_){

    const std::string sql =
        "CREATE TABLE IF NOT EXISTS two_view_geometries"
        "   (pair_id  INTEGER  PRIMARY KEY  NOT NULL,"
        "    rows     INTEGER               NOT NULL,"
        "    cols     INTEGER               NOT NULL,"
        "    data     BLOB,"
        "    config   INTEGER               NOT NULL,"
        "    F        BLOB,"
        "    E        BLOB,"
        "    H        BLOB);";
    SQLITE3_EXEC(*save_database_, sql.c_str(), nullptr);
}

void UpdateSchema(sqlite3** save_database_)
{
    if (!ExistsColumn("two_view_geometries", "F", *save_database_)) { 
        SQLITE3_EXEC(*save_database_,
                        "ALTER TABLE two_view_geometries ADD COLUMN F BLOB;", nullptr);
    }

    if (!ExistsColumn("two_view_geometries", "E", *save_database_)) {
        SQLITE3_EXEC(*save_database_,
                        "ALTER TABLE two_view_geometries ADD COLUMN E BLOB;", nullptr);
    }

    if (!ExistsColumn("two_view_geometries", "H", *save_database_)) {
        SQLITE3_EXEC(*save_database_,
                        "ALTER TABLE two_view_geometries ADD COLUMN H BLOB;", nullptr);
    }
    // Update user version number.
    std::unique_lock<std::mutex> lock(update_schema_mutex_);
    const std::string update_user_version_sql =
        StringPrintf("PRAGMA user_version = %d;", COLMAP_VERSION_NUMBER);
    SQLITE3_EXEC(*save_database_, update_user_version_sql.c_str(), nullptr);
}

void CreateTables(sqlite3** save_database_){
  CreateCameraTable(save_database_);
  CreateImageTable(save_database_);
  CreateKeypointsTable(save_database_);
  CreateDescriptorsTable(save_database_);
  CreateMatchesTable(save_database_);
  CreateTwoViewGeometriesTable(save_database_);
}

sqlite3* CreateNewSqlTable(const std::string& path){

    sqlite3* save_database_;

    SQLITE3_CALL(sqlite3_open_v2(
      path.c_str(), &save_database_,
      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX,
      nullptr));

    // Don't wait for the operating system to write the changes to disk
    SQLITE3_EXEC(save_database_, "PRAGMA synchronous=OFF", nullptr);

    // Use faster journaling mode
    SQLITE3_EXEC(save_database_, "PRAGMA journal_mode=WAL", nullptr);

    // Store temporary tables and indices in memory
    SQLITE3_EXEC(save_database_, "PRAGMA temp_store=MEMORY", nullptr);

    // Disabled by default
    SQLITE3_EXEC(save_database_, "PRAGMA foreign_keys=ON", nullptr);

    CreateTables(&save_database_);
    UpdateSchema(&save_database_);

    return save_database_;
}

void CopySqlCameraTable(sqlite3** save_database_, sqlite3_stmt * database_stmt, sqlite3_stmt * save_database_stmt){
    
    std::string sql = "INSERT INTO cameras(camera_id, model, width, height, params, " \
                    "prior_focal_length) VALUES(?, ?, ?, ?, ?, ?);";
    
    SQLITE3_CALL(sqlite3_prepare_v2(*save_database_, sql.c_str(), -1, &save_database_stmt, 0));
    //读取database_的cameras表单
    while (SQLITE3_CALL(sqlite3_step(database_stmt)) == SQLITE_ROW) {
        //cout << "dadad" << sqlite3_column_int64(database_stmt, 0) << endl;
        //写入CameraId
        SQLITE3_CALL(
            sqlite3_bind_int64(save_database_stmt, 1, sqlite3_column_int64(database_stmt, 0)));
        SQLITE3_CALL(sqlite3_bind_int64(save_database_stmt, 2, sqlite3_column_int64(database_stmt, 1)));
        SQLITE3_CALL(sqlite3_bind_int64(save_database_stmt, 3, sqlite3_column_int64(database_stmt, 2)));
        SQLITE3_CALL(sqlite3_bind_int64(save_database_stmt, 4, sqlite3_column_int64(database_stmt, 3)));

        // cout << "num_params_bytes:" << endl;
        const size_t num_params_bytes = static_cast<size_t>(sqlite3_column_bytes(database_stmt, 4));
        const size_t num_params = num_params_bytes / sizeof(double);

        std::vector<double> params_(8,0);
        memcpy(params_.data(), sqlite3_column_blob(database_stmt, 4),
                num_params_bytes);
        SQLITE3_CALL(sqlite3_bind_blob(save_database_stmt, 5, params_.data(),
                                        static_cast<int>(num_params_bytes),
                                        SQLITE_STATIC));
        
        SQLITE3_CALL(sqlite3_bind_int64(save_database_stmt, 6,
                                        sqlite3_column_int64(database_stmt, 5)));
        sqlite3_step(save_database_stmt);
        sqlite3_reset(save_database_stmt);
    }
    SQLITE3_CALL(sqlite3_reset(database_stmt));
    std::cout << "插入Cameras表单成功" << std::endl;
}

//读取database_的images表单
void CopySqlImageTable(sqlite3** save_database_, sqlite3_stmt * database_stmt, sqlite3_stmt * save_database_stmt){

    std::string sql = "INSERT INTO images(image_id, name, camera_id, prior_qw, prior_qx, " \
                    "prior_qy, prior_qz, prior_tx, prior_ty, prior_tz) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";

    SQLITE3_CALL(sqlite3_prepare_v2(*save_database_, sql.c_str(), -1, &save_database_stmt, 0));
    while (SQLITE3_CALL(sqlite3_step(database_stmt)) == SQLITE_ROW) {

        SQLITE3_CALL(sqlite3_bind_int64(save_database_stmt, 1, sqlite3_column_int64(database_stmt, 0)));
        std::string image_name = std::string(
                reinterpret_cast<const char*>(sqlite3_column_text(database_stmt, 1)));
        
        //cout << image_name << "\n";
        SQLITE3_CALL(sqlite3_bind_text(save_database_stmt, 2, image_name.c_str(),
                                static_cast<int>(image_name.size()),
                                SQLITE_STATIC));
        SQLITE3_CALL(sqlite3_bind_int64(save_database_stmt, 3, sqlite3_column_int64(database_stmt, 2)));

        // NaNs are automatically converted to NULLs in SQLite.
        SQLITE3_CALL(sqlite3_bind_double(save_database_stmt, 4, sqlite3_column_double(database_stmt, 3)));
        SQLITE3_CALL(sqlite3_bind_double(save_database_stmt, 5, sqlite3_column_double(database_stmt, 4)));
        SQLITE3_CALL(sqlite3_bind_double(save_database_stmt, 6, sqlite3_column_double(database_stmt, 5)));
        SQLITE3_CALL(sqlite3_bind_double(save_database_stmt, 7, sqlite3_column_double(database_stmt, 6)));

        // NaNs are automatically converted to NULLs in SQLite.
        SQLITE3_CALL(sqlite3_bind_double(save_database_stmt, 8, sqlite3_column_double(database_stmt, 7)));
        SQLITE3_CALL(sqlite3_bind_double(save_database_stmt, 9, sqlite3_column_double(database_stmt, 8)));
        SQLITE3_CALL(
            sqlite3_bind_double(save_database_stmt, 10, sqlite3_column_double(database_stmt, 9)));

        //SQLITE3_CALL(sqlite3_step(save_database_stmt));
        sqlite3_step(save_database_stmt);
        sqlite3_reset(save_database_stmt);
    }
    SQLITE3_CALL(sqlite3_reset(database_stmt));
    std::cout << "插入Images表单成功" << std::endl;
}