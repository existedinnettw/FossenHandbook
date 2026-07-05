from __future__ import annotations

import argparse
import html
import http.cookiejar
import re
import urllib.parse
import urllib.request
from pathlib import Path


FILES = {
    "Ch1.pdf": "https://www.dropbox.com/scl/fi/0t35ol7qjkbtq2y2tjtmr/Ch1.pdf?rlkey=733g820vq4vfxdn8pd9zl8xt9&dl=1",
    "Ch2.pdf": "https://www.dropbox.com/scl/fi/i7vlggjiipr8vpp6j1evo/Ch2.pdf?rlkey=w6o0au5nphl3h7y449tziior9&dl=1",
    "Ch3.pdf": "https://www.dropbox.com/scl/fi/szawrpaykezjodki8t3jq/Ch3.pdf?rlkey=54g895sql59x4gssoa7nextir&dl=1",
    "Ch4.pdf": "https://www.dropbox.com/scl/fi/td0dwv4cgjxmvdg8dhtg9/Ch4.pdf?rlkey=fcbxt5sa0kh826jk783tb7mqt&dl=1",
    "Ch5.pdf": "https://www.dropbox.com/scl/fi/7yuix37gbad59qma3hi1x/Ch5.pdf?rlkey=zi16uwq78d418v03lh1mdtfb9&dl=1",
    "Ch6.pdf": "https://www.dropbox.com/scl/fi/f2ipl6gxs9a826b6i06dw/Ch6.pdf?rlkey=utqhkxv7y353rittx1dnte0fo&dl=1",
    "Ch7.pdf": "https://www.dropbox.com/scl/fi/rth0nv12xsrf0ar8bsbwv/Ch7.pdf?rlkey=bjpndrhdfuxzc8281fx3n2hk8&dl=1",
    "Ch8.pdf": "https://drive.google.com/uc?export=download&id=1NEemmPazyFELrvziXi31iQZea1xYBPg9",
    "Ch9.pdf": "https://drive.google.com/uc?export=download&id=11A74KPYcGeAzU2rVSkWgHbi9CvvBesRa",
    "Ch10.pdf": "https://drive.google.com/uc?export=download&id=1-Dk0AE7UrPKBOPdeKazJSIPymbp8x6cR",
    "Ch11.pdf": "https://www.dropbox.com/scl/fi/sryr1340ilvjv9dpfs62t/Ch11.pdf?rlkey=grlws4qw8edl69s38dqivqfkp&dl=1",
    "Ch12.pdf": "https://www.dropbox.com/scl/fi/mf5ms14t9pp7mfe4qgf6w/Ch12.pdf?rlkey=odw8avz86a02w8tg6wnfmvu79&dl=1",
    "Ch13.pdf": "https://drive.google.com/uc?export=download&id=1K1RxgbNbvrErOX3CBtbcIcriACYm9Sm6",
    "Ch14.pdf": "https://www.dropbox.com/scl/fi/68ky3k4is0gahpxi3h010/Ch14.pdf?rlkey=6yq5djch0ycna9h69qz6xh4lz&dl=1",
    "Ch15.pdf": "https://drive.google.com/uc?export=download&id=1bJlyDYg5Le1PTa_0voSzYAO00ayzu9TA",
    "Ch16.pdf": "https://drive.google.com/uc?export=download&id=1QepdGD2JdRnh8tlzRaXnrugr78LAoxQz",
}


def build_opener() -> tuple[urllib.request.OpenerDirector, http.cookiejar.CookieJar]:
    cookiejar = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(cookiejar))
    opener.addheaders = [
        ("User-Agent", "Mozilla/5.0"),
        ("Accept", "*/*"),
    ]
    return opener, cookiejar


def download_dropbox(opener: urllib.request.OpenerDirector, url: str, dest: Path) -> tuple[int, str]:
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with opener.open(req, timeout=60) as resp:
        data = resp.read()
        ctype = resp.headers.get_content_type()
        if ctype == "text/html":
            text = data.decode("utf-8", "ignore")
            m = re.search(r'(?i)href="([^"]+dl=1[^"]*)"', text)
            if m:
                dl_url = html.unescape(m.group(1))
                if dl_url.startswith("/"):
                    dl_url = urllib.parse.urljoin(url, dl_url)
                req = urllib.request.Request(dl_url, headers={"User-Agent": "Mozilla/5.0"})
                with opener.open(req, timeout=60) as resp2:
                    data = resp2.read()
                    dest.write_bytes(data)
                    return len(data), resp2.headers.get_content_type()
        dest.write_bytes(data)
        return len(data), ctype


def download_google_drive(
    opener: urllib.request.OpenerDirector,
    cookiejar: http.cookiejar.CookieJar,
    url: str,
    dest: Path,
) -> tuple[int, str]:
    parsed = urllib.parse.urlparse(url)
    file_id = urllib.parse.parse_qs(parsed.query).get("id", [""])[0]
    current = url
    confirm = None

    for _ in range(3):
        req = urllib.request.Request(current, headers={"User-Agent": "Mozilla/5.0"})
        with opener.open(req, timeout=60) as resp:
            data = resp.read()
            ctype = resp.headers.get_content_type()
            if ctype != "text/html" and data[:4] == b"%PDF":
                dest.write_bytes(data)
                return len(data), ctype

            text = data.decode("utf-8", "ignore")
            m = re.search(r"confirm=([0-9A-Za-z_]+)", text)
            if m:
                confirm = m.group(1)
            else:
                for cookie in cookiejar:
                    if cookie.name.startswith("download_warning"):
                        confirm = cookie.value
                        break
            if not confirm:
                if data[:4] == b"%PDF":
                    dest.write_bytes(data)
                    return len(data), ctype
                raise RuntimeError(f"Could not get confirm token for {url}")

            current = f"https://drive.google.com/uc?export=download&confirm={confirm}&id={file_id}"

    raise RuntimeError(f"Failed after retries for {url}")


def download_file(
    opener: urllib.request.OpenerDirector,
    cookiejar: http.cookiejar.CookieJar,
    url: str,
    dest: Path,
) -> tuple[int, str]:
    if "dropbox.com" in url:
        return download_dropbox(opener, url, dest)
    if "drive.google.com" in url:
        return download_google_drive(opener, cookiejar, url, dest)
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with opener.open(req, timeout=60) as resp:
        data = resp.read()
        dest.write_bytes(data)
        return len(data), resp.headers.get_content_type()


def main() -> int:
    parser = argparse.ArgumentParser(description="Download chapters 1-16 as PDFs into a directory.")
    parser.add_argument("-o", "--out-dir", default="out", help="Output directory (default: out)")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    opener, cookiejar = build_opener()

    for name, url in FILES.items():
        dest = out_dir / name
        size, ctype = download_file(opener, cookiejar, url, dest)
        print(f"{name}: {size} bytes ({ctype})")

    print(f"done: {len(FILES)} files in {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
