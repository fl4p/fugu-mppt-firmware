import ftplib
import os
import re

from etc.fugu.discover import discover_scope_servers


def _download_ftp_file(ftp_handle:ftplib.FTP, name, dest, overwrite):
    """ downloads a single file from an ftp server """
    d = os.path.dirname(dest)
    d and os.makedirs(d, exist_ok=True)

    if not os.path.exists(dest) or overwrite is True:
        try:
            with open(dest, 'wb') as f:
                ftp_handle.retrbinary("RETR {0}".format(name), f.write)
            print("downloaded: {0}".format(dest))
        except FileNotFoundError:
            print("FAILED: {0}".format(dest))
    else:
        print("already exists: {0}".format(dest))

def _upload_ftp_file(ftp_handle:ftplib.FTP, name, src, overwrite=True):
    #d = os.path.dirname(dest)
    #d and os.makedirs(d, exist_ok=True)

    # ftp_handle.sendcmd('MLST')
    assert overwrite, "not implemented"
    if overwrite is True or not os.path.exists(dest):
        try:
            with open(src, 'rb') as f:
                ftp_handle.storbinary('STOR %s' % name, f)
            print("uploaded: {0}".format(src))
        except FileNotFoundError:
            print("FAILED: {0}".format(src))
    else:
        print("already exists: {0}".format(src))


def _file_name_match_pattern(pattern, name):
    return not pattern or bool(re.match(pattern, name))


def _dl_recurse(ftp_handle:ftplib.FTP, name, overwrite, guess_by_extension, pattern):
    """ replicates a directory on an ftp server recursively """
    ftp_handle.cwd(name)
    for item, attr in ftp_handle.mlsd(name):
        m, n = item.split('\t')
        if list(attr.keys())[0][0] == 'd':  # _is_ftp_dir(ftp_handle, n, guess_by_extension):
            _dl_recurse(ftp_handle, n, overwrite, guess_by_extension, pattern)
        else:
            if _file_name_match_pattern(pattern, name):
                _download_ftp_file(ftp_handle, n, os.path.join(name, n), overwrite)
    ftp_handle.cwd('..')


def download_ftp_tree(ftp_handle:ftplib.FTP, path, destination, pattern=None, overwrite=False, guess_by_extension=True):
    """
    Downloads an entire directory tree from an ftp server to the local destination
    :param ftp_handle: an authenticated ftplib.FTP instance
    :param path: the folder on the ftp server to download
    :param destination: the local directory to store the copied folder
    :param pattern: Python regex pattern, only files that match this pattern will be downloaded.
    :param overwrite: set to True to force re-download of all files, even if they appear to exist already
    :param guess_by_extension: It takes a while to explicitly check if every item is a directory or a file.
        if this flag is set to True, it will assume any file ending with a three character extension ".???" is
        a file and not a directory. Set to False if some folders may have a "." in their names -4th position.
    """
    path = path.lstrip("/")
    original_directory = os.getcwd()  # remember working directory before function is executed
    os.makedirs(destination, exist_ok=True)
    os.chdir(destination)  # change working directory to ftp mirror directory

    _dl_recurse(
        ftp_handle,
        path,
        pattern=pattern,
        overwrite=overwrite,
        guess_by_extension=guess_by_extension)

    os.chdir(original_directory)  # reset working directory to what it was before function exec


if __name__ == "__main__":
    hosts = discover_scope_servers()
    print(hosts)
    if not hosts:
        print('no hosts discovered on network')

    upload = False

    if upload:
        input('confirm upload to %d devices ' % len(hosts))

    for addr,_,name in hosts:
        print(addr, name)
        username = "user"
        password = "password"
        remote_dir = ""
        local_dir = "dl/"+name+addr
        pattern = r".*"  # .*\.jpg$"
        ftp = ftplib.FTP(addr, username, password, timeout=4)
        download_ftp_tree(ftp, remote_dir, local_dir, pattern=pattern, overwrite=True, guess_by_extension=True)
