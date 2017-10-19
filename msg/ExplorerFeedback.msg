# Corresponding Action id
int32 id

# Status (as listed in rsa17/explorer.hpp)
int32   STATUS_PAUSED   = 0
int32   STATUS_RUNNING  = 1
int32   STATUS_ARRIVED  = 2
int32   STATUS_BLOCKED  = 3
int32 status

# Keep updated on continuing state of exploration
# Search strategy uses mode numbers
bool   operating
int32  searchStrategy
