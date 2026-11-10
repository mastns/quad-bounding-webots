"""Draws the controllers' supervisor labels and the engine badge on rendered MuJoCo frames.

Webots draws a label as Webots R2025a movies show it (measured on this machine): the line box of the
text (font ascent + descent) is 0.5 * size * view height tall, its top-left corner is at
(x * width, y * height), the text starts after a small padding, and the font comes from
WEBOTS_HOME/resources/fonts. The same TTF files are used here when Webots is installed, Liberation
or DejaVu fonts otherwise, so the HUD of a MuJoCo video matches the one of a Webots video.
"""
import os

from PIL import Image, ImageDraw, ImageFont

WEBOTS_FONTS = os.path.join(os.environ.get('WEBOTS_HOME', os.path.expanduser('~/webots')), 'resources', 'fonts')
SYSTEM_FONT_DIRS = ['/usr/share/fonts/truetype/liberation', '/usr/share/fonts/truetype/liberation2',
                    '/usr/share/fonts/truetype/dejavu']
FALLBACKS = {'Arial Black': ('LiberationSans-Bold.ttf', 'DejaVuSans-Bold.ttf'),
             'Lucida Console': ('LiberationMono-Regular.ttf', 'DejaVuSansMono.ttf'),
             'Arial': ('LiberationSans-Regular.ttf', 'DejaVuSans.ttf')}
BADGE = dict(font='Arial Black', size=0.085, color=0x37474F)   # as the "Webots" badge of tools/movie_recorder.py
MARGIN = 0.012                                                  # as the HUD's margin, in view fractions


def font_file(name):
    """TTF for a Webots font family name: Webots' own copy, else a metric-compatible system font."""
    path = os.path.join(WEBOTS_FONTS, name + '.ttf')
    if os.path.exists(path):
        return path
    for candidate in FALLBACKS.get(name, FALLBACKS['Arial']):
        for folder in SYSTEM_FONT_DIRS:
            path = os.path.join(folder, candidate)
            if os.path.exists(path):
                return path
    raise FileNotFoundError('no TTF found for the label font "%s" (looked in %s and %s)'
                            % (name, WEBOTS_FONTS, ', '.join(SYSTEM_FONT_DIRS)))


class Hud:
    def __init__(self, width, height, badge='MuJoCo'):
        self.w, self.h, self.badge = width, height, badge
        self._line_per_em = {}   # font file -> (ascent + descent) / em
        self._fonts = {}         # (font file, px) -> ImageFont

    def font(self, name, size):
        """The font for a label of Webots `size`: a line box 0.5 * size * height pixels tall."""
        path = font_file(name)
        if path not in self._line_per_em:
            ascent, descent = ImageFont.truetype(path, 100).getmetrics()
            self._line_per_em[path] = (ascent + descent) / 100.0
        px = max(1, int(round(0.5 * size * self.h / self._line_per_em[path])))
        if (path, px) not in self._fonts:
            self._fonts[path, px] = ImageFont.truetype(path, px)
        return self._fonts[path, px]

    def draw(self, frame, labels):
        """frame: height x width x 3 uint8 array; labels: {id: dict(text, x, y, size, color, transparency, font)}.
        Returns a PIL image with the labels and the badge."""
        image = Image.fromarray(frame)
        draw = ImageDraw.Draw(image, 'RGBA')
        for label in labels.values():
            if not label['text']:
                continue
            color = int(label['color'])
            alpha = int(round(255 * (1.0 - min(1.0, max(0.0, float(label['transparency']))))))
            padding = 0.05 * label['size'] * self.h
            draw.text((label['x'] * self.w + padding, label['y'] * self.h), label['text'],
                      font=self.font(label['font'], label['size']), anchor='la',
                      fill=((color >> 16) & 255, (color >> 8) & 255, color & 255, alpha))
        if self.badge:
            c = BADGE['color']
            draw.text((self.w * (1 - MARGIN), self.h * (1 - MARGIN)), self.badge, anchor='rd',
                      font=self.font(BADGE['font'], BADGE['size']), fill=((c >> 16) & 255, (c >> 8) & 255, c & 255, 255))
        return image
