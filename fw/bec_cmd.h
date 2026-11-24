/*
 * This is free and unencumbered software released into the public domain.
 * See the LICENSE file for additional details.
 *
 * Designed by Chris Hooper in 2025.
 *
 * ---------------------------------------------------------------------
 *
 * AmigaPCI command interface
 */

#ifndef _BEC_CMD_H
#define _BEC_CMD_H

/*
 * The BEC message interface is implemented on top of either the Ricoh
 * RP5C01 emulation or Amiga keyboard emulation. It provides a way for
 * AmigaOS programs to interact with the board management STM32 processor.
 *
 * All messages and replies begin with a magic sequence and end with a
 * 4-byte CRC. The maximum length of a message payload is
 * BEC_MSG_MAX_PAYLOAD (1024) bytes, which bounds the entire message to
 * 1033 bytes. The firmware updater accepts payloads of at most 271 bytes.
 *
 * Message sequence (all data is in nibbles, high nibble first)
 *     Magic   0xc 0xd 0x6 0x8
 *     Command X X               Upper 2 bits are command options
 *     Length  X X X X           Payload length doesn't include header or CRC
 *     Payload [ X X * ]         Even number of nibbles in optional payload
 *     CRC     X X X X X X X X   32-bit in big endian format (includes Cmd+Len)
 */

/* Command codes sent to AmigaPCI STM32 */
#define BEC_CMD_NULL         0x00  // Do nothing (no reply)
#define BEC_CMD_NOP          0x01  // Do nothing but reply (success)
#define BEC_CMD_ID           0x02  // Send STM32 firmware ID and configuration
#define BEC_CMD_UPTIME       0x03  // Send STM32 uptime in microseconds (64-bit)
#define BEC_CMD_TESTPATT     0x05  // Send test pattern
#define BEC_CMD_LOOPBACK     0x06  // Reply with (exact) sent message
#define BEC_CMD_CONS_OUTPUT  0x07  // Receive STM32 console output
#define BEC_CMD_CONS_INPUT   0x08  // Send STM32 console input keystrokes
#define BEC_CMD_SET          0x09  // Set config value
#define BEC_CMD_GET          0x0a  // Get config value
#define BEC_CMD_SET_MAP      0x0b  // Set map (keyboard, mouse, etc, macros)
#define BEC_CMD_GET_MAP      0x0c  // Get map (keyboard, mouse, etc, macros)
#define BEC_CMD_POLL_INPUT   0x0d  // Capture input (such as keystrokes)
#define BEC_CMD_SENSORS      0x0e  // Get sensor reports
#define BEC_CMD_CONTROL      0x0f  // Board control (power / reset)
#define BEC_CMD_FW_UNLOCK    0x10  // Allow firmware update erase / write
#define BEC_CMD_FW_LOCK      0x11  // Disable firmware update erase / write
#define BEC_CMD_FW_READ      0x12  // Read section of firmware
#define BEC_CMD_FW_WRITE     0x13  // Write section of firmware
#define BEC_CMD_FAN          0x18  // Fan settings
#define BEC_CMD_FOPEN        0x20  // File storage open
#define BEC_CMD_FCLOSE       0x21  // File storage close
#define BEC_CMD_FREAD        0x22  // File storage read
#define BEC_CMD_FWRITE       0x23  // File storage write
#define BEC_CMD_FSEEK        0x24  // File storage seek
#define BEC_CMD_FCREATE      0x25  // File storage create
#define BEC_CMD_FDELETE      0x26  // File storage delete
#define BEC_CMD_FRENAME      0x27  // File storage rename
#define BEC_CMD_FPATH        0x28  // File storage get path to handle
#define BEC_CMD_FSETPERMS    0x29  // File storage set permissions
#define BEC_CMD_FSETOWN      0x2a  // File storage set owner / group
#define BEC_CMD_FSETDATE     0x2b  // File storage set date

/* Status codes returned by AmigaPCI STM32 */
#define BEC_STATUS_OK        0x00  // Success
#define BEC_STATUS_FAIL      0x01  // Generic failure
#define BEC_STATUS_LOOPBACK  0x02  // Special loopback return code
#define BEC_STATUS_UNKCMD    0x03  // Unknown command
#define BEC_STATUS_BADARG    0x04  // Bad command argument
#define BEC_STATUS_BADLEN    0x05  // Bad message length
#define BEC_STATUS_NODATA    0x06  // No data available
#define BEC_STATUS_LOCKED    0x07  // Resource locked
#define BEC_STATUS_TIMEOUT   0x08  // Timeout talking with BEC
#define BEC_STATUS_BADMAGIC  0x09  // Bad magic in response message
#define BEC_STATUS_REPLYLEN  0x0a  // Response message is too long
#define BEC_STATUS_REPLYCRC  0x0b  // Response message has bad CRC
#define BEC_STATUS_CRC       0x0c  // CRC failure
#define BEC_STATUS_EOF       0x0d  // End of file (or directory) reached
#define BEC_STATUS_PERM      0x0e  // Permission failure
#define BEC_STATUS_INVALID   0x0f  // Invalid mode for operation
#define BEC_STATUS_NOTEMPTY  0x10  // Directory not empty
#define BEC_STATUS_NOEXIST   0x11  // Object does not exist
#define BEC_STATUS_EXIST     0x12  // Object already exists
#define BEC_STATUS_UNAVAIL   0x13  // Service (storage) not available
#define BEC_STATUS_INUSE     0x14  // Object in use (locked)
#define BEC_STATUS_STALE     0x15  // Handle unknown (BEC restarted or
                                   // storage removed)

#define BEC_MSG_HDR_LEN 5  // Number of bytes in Magic + cmd + length
#define BEC_MSG_CRC_LEN 4  // Number of bytes in CRC
#define BEC_MSG_MAX_PAYLOAD 1024  // Maximum message payload length

/*
 * The below structure is a response to the following command:
 *    BEC_CMD_ID
 */
typedef struct {
    uint16_t bid_version[2];       // BEC firmware version    (major-minor)
    uint8_t  bid_date[4];          // BEC firmware build date (cc-yy-mm-dd)
    uint8_t  bid_time[4];          // BEC firmware build time (hh-mm-ss-00)
    char     bid_serial[24];       // BEC serial number
    uint16_t bid_features;         // Available features
    uint16_t bid_rev;              // Protocol revision (00.01)
    char     bid_name[16];         // Unique name for this board
    uint8_t  bid_board_type;       // Board type (0=APCI, 1=APCIDev, 2=KeyJAM)
    uint8_t  bid_board_rev;        // Board revision
    uint8_t  bid_unused1[2];       // Unused space
    uint32_t bid_size;             // Firmware total size in bytes
    uint8_t  bid_unused[24];       // Unused space
} bec_id_t;

#define BID_FEATURE_CAN_FW_UPDATE 0x0001  // Can do firmware update
#define BID_FEATURE_FW_UPDATE     0x0002  // Running firmware updater

/*
 * The below structure is used for request / response of the following commands:
 *    BEC_CMD_GET_MAP
 *    BEC_CMD_SET_MAP
 */
typedef struct {
    uint8_t  bkm_which;            // Which keymap (see BKM_WHICH_*)
    uint8_t  bkm_start;            // First entry number
    uint8_t  bkm_len;              // Size of single entry
    uint8_t  bkm_count;            // Count of entries (0 = no entries)
} bec_keymap_t;

#define BKM_WHICH_KEYMAP          0x01  // Scancode mapping table
#define BKM_WHICH_BUTTONMAP       0x02  // Mouse button map
#define BKM_WHICH_MOUSE_SPEED     0x08  // Mouse speed factors
#define BKM_WHICH_TMOUSE_SPEED    0x09  // Temporary Mouse speed factors
#define BKM_WHICH_DEF_KEYMAP      0x11  // Default scancode mapping table
#define BKM_WHICH_DEF_BUTTONMAP   0x12  // Default button mapping table

/*
 * The below structure is used for the following command:
 *    BEC_CMD_POLL_INPUT
 *
 * The reply includes a buffer of value pairs which immediately follow
 * the reply structure. The first of each pair is the scancode, and
 * the second is 0 for key down and 1 for key up.
 */
typedef struct {
    uint8_t  bp_source;            // Which input to poll
    uint8_t  bp_count;             // Count of value pairs
    uint16_t bp_timeout;           // Timeout in milliseconds
} bec_poll_t;

#define BP_SOURCE_NONE           0x00  // Disable polling
#define BP_SOURCE_HID_SCANCODE   0x01  // Lightly processed HID scancodes
#define BP_SOURCE_AMIGA_SCANCODE 0x02  // Key scancodes to be sent to Amiga

/*
 * The below structure is used for the following commands:
 *    BEC_CMD_FW_READ
 *    BEC_CMD_FW_WRITE
 */
typedef struct {
    uint32_t bfrw_offset;          // Offset for read or write
    uint16_t bfrw_size;            // Number of bytes to read or write
    uint8_t  bfrw_unused[6];       // Unused space
} bec_fw_rw_t;

/*
 * The below structure is used for the following command:
 *    BEC_CMD_SENSORS
 *
 * The request includes a byte which is the first sensor to retrieve in
 * this request. The reply is an array of bec_sensors_t. The sensor count
 * is the reply length divided by sizeof (bec_sensors_t). An empty reply
 * means there are no more sensors.
 */
typedef struct {
    char     bs_name[12];          // Sensor name
    int32_t  bs_reading;           // Sensor reading in milli-units
    int32_t  bs_warn_lo;           // Sensor minimum in milli-units
    int32_t  bs_warn_hi;           // Sensor maximum in milli-units
    uint8_t  bs_type;              // Sensor type (V C R or %)
    uint8_t  bs_domain;            // Sensor power domain (0=all, 1=PS, 2=regs)
    uint16_t bs_state;             // Sensor state (under/over limit, etc)
    uint8_t  bs_unused[8];         // Unused space
} bec_sensors_t;

#define BS_STATE_UNDER_LIMIT        0x0001  // Sensor is under limit
#define BS_STATE_WARNED_OVER_LIMIT  0x0002  // Sensor warned under limit
#define BS_STATE_OVER_LIMIT         0x0004  // Sensor is over limit
#define BS_STATE_WARNED_UNDER_LIMIT 0x0008  // Sensor warned over limit
#define BS_STATE_IGNORED            0x0010  // Sensor is ignored

/*
 * The below definitions are used for the following command:
 *    BEC_CMD_CONTROL
 *
 * The request includes a byte which is the operation to perform, one
 * of the BC_* listed below.
 *
 * bcg_state_t is the BC_GET reply structure
 */
#define BC_POWER_CYCLE              0x01    // Power cycle Amiga
#define BC_POWER_OFF                0x02    // Power off Amiga
#define BC_RESET_AMIGA              0x03    // Reset Amiga
#define BC_RESET_AMIGA_LONG         0x04    // Reset Amiga (long)
#define BC_RESET_BEC                0x05    // Reset BEC
#define BC_RESET_USB                0x06    // Reset BEC USB ports
#define BC_POWER_OFF_USB            0x07    // Power off BEC USB ports
#define BC_POWER_ON_USB             0x08    // Power on BEC USB ports
#define BC_POWER_AC_ON              0x09    // Power on when AC restored
#define BC_POWER_AC_OFF             0x0a    // Remain off when AC restored
#define BC_GET                      0x10    // Get BEC state

typedef struct {
    uint8_t  bcg_power_state;       // System power state
    uint8_t  bcg_usb_power_state;   // USB power state
    uint8_t  bcg_usb_keyboards;     // Connected USB keyboard count
    uint8_t  bcg_usb_mice;          // Connected USB mouse count
    uint8_t  bcg_usb_joysticks;     // Connected USB joystick count
    uint8_t  bcg_hid_enabled;       // HID mouse override enabled
    uint8_t  bcg_amiga_kbd_state;   // Amiga keyboard state
    uint8_t  bcg_unused[9];         // Unused space
} bcg_state_t;

#define BGC_POWER_STATE_INITIAL      0x00   // Power state is initiallizing
#define BGC_POWER_STATE_POWERING_ON  0x01   // Powering on
#define BGC_POWER_STATE_POWERING_OFF 0x02   // Powering off
#define BGC_POWER_STATE_CYCLE        0x03   // Cycling power
#define BGC_POWER_STATE_ON           0x04   // Power supply is on
#define BGC_POWER_STATE_OFF          0x05   // Power supply is off
#define BGC_POWER_STATE_FAULT        0x06   // Power supply has faulted
#define BGC_POWER_STATE_FAULT_ON     0x07   // Power supply failed to power on
#define BGC_POWER_STATE_FAULT_OFF    0x08   // Power supply failed to power off

#define BCG_USB_POWERED_MASK_USB0    0x01   // Supplying power to USB0
#define BCG_USB_POWERED_MASK_USB1    0x02   // Supplying power to USB1

#define BCG_AKBD_STATE_LOST_SYNC     0x01   // Lost keyboard sync with Amiga
#define BCG_AKBD_STATE_HAVE_SYNC     0x02   // Have keyboard sync with Amiga
#define BCG_AKBD_STATE_SENT_WAKE     0x04   // Sent wake to Amiga
#define BCG_AKBD_STATE_CAPTURE       0x08   // Keystrokes being captured


/*
 * The below structure is used for the following command:
 *    BEC_CMD_FAN
 *
 * bf_fan_t is the request structure
 *     bf_fan_op is the operation to perform, one of BF_* below.
 *     bf_unused must be 0.
 *     bf_value is the value to set.
 *
 * bf_fan_settings_t is the BF_GET reply structure
 */
typedef struct {
    uint8_t  bf_fan_op;            // Fan operation (BF_* below)
    uint8_t  bf_unused;            // Unused space
    uint16_t bf_value;             // Value to set
} bec_fan_t;

typedef struct {
    uint8_t  bfs_fan_speed;        // Fan speed percent (0-100, BIT(7)=auto)
    uint8_t  bfs_fan_speed_min;    // Fan minimum speed percent
    uint16_t bfs_fan_rpm_max;      // Fan maximum RPM
    uint8_t  bfs_temp_min;         // Temp C for minimum fan speed
    uint8_t  bfs_temp_max;         // Temp C for maximum fan speed
    uint8_t  bfs_unused[6];        // Unused space
} bec_fan_settings_t;

/* Fan speed bec_fan_t bf_fan_op */
#define BF_SET_SPEED_AUTO           0x01    // Set fan speed automatically
#define BF_SET_SPEED_SET            0x02    // Set fan speed to a fixed percent
#define BF_SET_SPEED_MIN            0x03    // Set minimum fan percent
#define BF_SET_RPM_MAX              0x04    // Set maximum fan RPM
#define BF_SET_TEMP_MIN             0x05    // Set temp C for minimum fan
#define BF_SET_TEMP_MAX             0x06    // Set temp C for maximum fan
#define BF_GET                      0x10    // Get fan settings


/*
 * File operations
 *
 * These mirror the KickSmash hostsmash file service (KM_OP_F*), with the
 * same structures less the KickSmash message header. Files are served
 * from USB mass storage (FAT12/16/32 or exFAT) attached to the BEC.
 *
 * Handles are opaque 32-bit values. Handle 0 is the Volume Directory,
 * which lists one volume per USB storage device. Handle 0xffffffff is
 * the root of the first USB storage volume. Handles of a previous BEC
 * run, or of a USB device which was removed, get BEC_STATUS_STALE.
 *
 * Volume names are the FAT volume label, or USB<n> if the volume has no
 * label (or its label is already in use). USB<n>: is also always
 * accepted for device n. Names are ISO-8859-1 (Latin-1).
 *
 * Every request may get a reply status of BEC_STATUS_UNAVAIL if the BEC
 * firmware was built without file services.
 */

typedef uint32_t bf_handle_t;

/* bf_fopenhandle_t bf_type and bf_fdirent_t bfd_type */
#define BF_TYPE_ANY         0x0000  // Any type of file (for open)
#define BF_TYPE_UNKNOWN     0x0000  // Unknown (for reported type)
#define BF_TYPE_FILE        0x0001  // Regular file
#define BF_TYPE_DIR         0x0002  // Directory
#define BF_TYPE_LINK        0x0003  // Symbolic (soft) link (unsupported)
#define BF_TYPE_HLINK       0x0004  // Hard link (unsupported)
#define BF_TYPE_VOLUME      0x000a  // Disk volume
#define BF_TYPE_VOLDIR      0x000b  // Volume directory

/* bf_fopenhandle_t bf_mode (see KickSmash HM_MODE_*) */
#define BF_MODE_READ        0x0001  // Read
#define BF_MODE_WRITE       0x0002  // Write
#define BF_MODE_RDWR        0x0003  // Read/write
#define BF_MODE_APPEND      0x0004  // Append to file
#define BF_MODE_CREATE      0x0100  // Create file if it doesn't exist
#define BF_MODE_TRUNC       0x0200  // Truncate file at open
#define BF_MODE_DIR         0x0800  // Read directory entry in parent (stat)
#define BF_MODE_READDIR     0x0801  // Read directory (composite)
#define BF_MODE_NOFOLLOW    0x1000  // Do not follow symlink on READDIR
#define BF_MODE_LINK        0x2000  // Symlink
#define BF_MODE_READLINK    0x2001  // Read symlink (composite)
#define BF_MODE_LOCK        0x4000  // Hold shared lock on object while open
#define BF_MODE_LOCK_EX     0xc000  // Hold exclusive lock (composite)
/*
 * Lock modes: an open with BF_MODE_LOCK holds a shared lock on the object
 * until closed. BF_MODE_LOCK_EX holds an exclusive lock. The BEC refuses
 * with BEC_STATUS_INUSE an exclusive lock on an object which has any
 * lock, a shared lock on an object which has an exclusive lock, and a
 * delete of an object which is open. The BEC sets BF_MODE_LOCK in
 * bf_mode of every successful BEC_CMD_FOPEN reply.
 *
 * A file may be open for write by only one handle at a time.
 */

/* bf_freadwrite_t bf_flag */
#define BF_FLAG_SEEK0       0x0001  // Seek the start of file before read

/* Maximum data bytes in a BEC_CMD_FREAD reply or BEC_CMD_FWRITE request */
#define BF_RW_MAX (BEC_MSG_MAX_PAYLOAD - sizeof (bf_freadwrite_t))

/*
 * The below structure is used for the following commands:
 *    BEC_CMD_FDELETE  (the filename immediately follows this struct)
 *    BEC_CMD_FPATH    (the reply path immediately follows this struct)
 */
typedef struct {
    bf_handle_t  bf_handle;  // Handle or parent dir handle
} bf_fhandle_t;

/*
 * The below structure is used for the following commands:
 *    BEC_CMD_FOPEN
 *    BEC_CMD_FCLOSE
 *    BEC_CMD_FCREATE
 *    BEC_CMD_FSETPERMS
 */
typedef struct {
    bf_handle_t  bf_handle;  // Parent dir handle on open, new handle on reply
    uint16_t     bf_type;    // File or directory type (from BEC)
    uint16_t     bf_mode;    // File mode for open
    uint32_t     bf_aperms;  // Amiga file permissions for create
    /* For open, the filename immediately follows this struct */
} bf_fopenhandle_t;

/*
 * The below structure is used for the following commands:
 *    BEC_CMD_FREAD
 *    BEC_CMD_FWRITE
 */
typedef struct {
    bf_handle_t  bf_handle;  // File handle for request
    uint32_t     bf_length;  // Length of request or reply data size
    uint16_t     bf_flag;    // Read / write operation flags (BF_FLAG_*)
    uint16_t     bf_unused;  // Unused
    /* For write, the data immediately follows this struct */
} bf_freadwrite_t;

/*
 * The below structure is used for the following command:
 *    BEC_CMD_FRENAME
 */
typedef struct {
    bf_handle_t  bf_shandle; // Source parent dir handle
    bf_handle_t  bf_dhandle; // Destination parent dir handle
    /* Source and destination filenames immediately follow this struct */
} bf_frename_t;

/*
 * The below structure is used for the following command:
 *    BEC_CMD_FSEEK
 */
typedef struct {
    bf_handle_t  bf_handle;  // File handle for request
    uint32_t     bf_off_hi;  // New file offset upper 32 bits (signed)
    uint32_t     bf_off_lo;  // New file offset lower 32 bits
    uint32_t     bf_old_hi;  // Reply: Prev file offset upper 32 bits (signed)
    uint32_t     bf_old_lo;  // Reply: Prev file offset lower 32 bits
    int8_t       bf_seek;    // -1=from beginning, 0=from current, 1=from end
    uint8_t      bf_flags;   // FT_SEEK_FLAG_*
    uint16_t     bf_unused2; // Unused
} bf_fseek_t;

/* bf_fseek_t bf_flags */
#define FT_SEEK_FLAG_NOEXTEND 0x01  // Fail seek before start or past EOF

/*
 * The below structure is used for the following command:
 *    BEC_CMD_FSETDATE
 */
typedef struct {
    bf_handle_t  bf_handle;  // Parent dir handle
    uint8_t      bf_which;   // 0=set modify, 1=get modify, 2=set change,
                             // 3=get change, 4=set access, 5=get access
    uint8_t      bf_unused0; // Unused
    uint16_t     bf_unused1; // Unused
    uint32_t     bf_time;    // Time secs since Jan 1, 1970
    uint32_t     bf_time_ns; // Time nanoseconds
    /* The filename immediately follows this struct */
} bf_fsetdate_t;

/*
 * The below structure is used for the following command:
 *    BEC_CMD_FSETOWN
 */
typedef struct {
    bf_handle_t  bf_handle;  // Parent dir handle
    uint32_t     bf_oid;     // New owner ID
    uint32_t     bf_gid;     // New group ID
    /* The filename immediately follows this struct */
} bf_fsetown_t;

/*
 * The below structure is used for the following command:
 *    BEC_CMD_FREAD (only for directories)
 */
typedef struct {
    uint16_t     bfd_type;    // File or directory type
    uint16_t     bfd_elen;    // Entry length
    uint32_t     bfd_size_hi; // file size upper 32 bits
    uint32_t     bfd_size_lo; // file size lower 32 bits
    uint32_t     bfd_blksize; // disk block size
    uint32_t     bfd_blks;    // disk blocks consumed lower 32 bits
    uint32_t     bfd_atime;   // Access time   (secs since Jan 1, 1970)
    uint32_t     bfd_ctime;   // Creation time (secs since Jan 1, 1970)
    uint32_t     bfd_mtime;   // Modify time   (secs since Jan 1, 1970)
    uint32_t     bfd_aperms;  // Amiga-style file permissions
    uint32_t     bfd_ino;     // Unique file number per-filesystem
                              // Volume entries: AV_FLAG_* flags
    uint32_t     bfd_ouid;    // Owner userid
    uint32_t     bfd_ogid;    // Owner groupid
    uint32_t     bfd_mode;    // Unix disk mode (permissions)
    uint32_t     bfd_nlink;   // Filesystem links to file
                              // Volume entries: boot priority (signed)
    uint32_t     bfd_rdev;    // Filesystem block and char devices
                              // Volume entries: BEC epoch (changes when
                              // the BEC restarts)
    uint32_t     bfd_rsvd[2]; // Reserved space for future expansion
    /* Name and comment follows... */
} bf_fdirent_t;

/* Volume entry flags (bfd_ino of a Volume Directory entry) */
#define BF_AV_FLAG_BOOTABLE 0x01  // Volume is bootable at bfd_nlink priority

/*
 * The filename immediately follows bf_fdirent_t in the reply
 * message, and the file comment immediately follows the filename
 * (each NIL-terminated). The next struct in the directory list
 * will be two-byte aligned following the comment.
 */

#endif  /* _BEC_CMD_H */
