import lzma, os, shutil, threading, time
from logging.handlers import RotatingFileHandler


class ArchivingRotatingFileHandler(RotatingFileHandler):
    """RotatingFileHandler that compresses the oldest backup into archive_dir instead of deleting it."""

    def __init__(self, filename, archive_dir, **kw):
        super().__init__(filename, **kw)
        self.archive_dir = archive_dir
        os.makedirs(archive_dir, exist_ok=True)

    def doRollover(self):
        last = f"{self.baseFilename}.{self.backupCount}"
        if self.backupCount > 0 and os.path.exists(last):
            with open(last, 'rb') as f:
                head = f.read(19).decode('ascii', 'replace').replace(' ', '_').replace(':', '')
            base = os.path.basename(self.baseFilename)
            stem = os.path.join(self.archive_dir, f"{base}.{head or int(time.time())}")
            n = 0
            while any(os.path.exists(f"{stem}.{n}{x}") for x in ('.pending', '.xz', '.xz.tmp')):
                n += 1
            pending = f"{stem}.{n}.pending"
            os.rename(last, pending)
            threading.Thread(target=self._compress, args=(pending,), daemon=False).start()
        super().doRollover()

    @staticmethod
    def _compress(pending):
        out = pending[:-len('.pending')] + '.xz'
        with open(pending, 'rb') as src, lzma.open(out + '.tmp', 'wb', preset=6) as dst:
            shutil.copyfileobj(src, dst)
        os.replace(out + '.tmp', out)
        os.remove(pending)
