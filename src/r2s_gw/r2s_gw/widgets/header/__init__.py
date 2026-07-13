from textual.widget import Widget
from textual.widgets import Static

LOGO = r"""   ___  __  __         ___           _
  / _ |/ /_/ /__ ____ / _ \___ _____(_)__  ___ _
 / __ / __/ / _ `(_-</ , _/ _ `/ __/ / _ \/ _ `/
/_/ |_\__/_/\_,_/___/_/|_|\_,_/\__/_/_//_/\_, /
                                         /___/  """


class Header(Widget):
    DEFAULT_CSS = """
    Header {
      border: none;
      border-title-align: center;
      grid-columns: 20 1fr 60;
      height: 8;
      grid-size: 3;
      layout: grid;
    }

    .header-box {
      margin: 0 0 0 3;
      text-align: left;
    }

    .header-right {
        color: #76b900;
    }
    """

    def left(self):
        return Static("", classes="header-box")

    def center(self):
        return Static("", classes="header-box")

    def right(self):
        return Static(LOGO, classes="header-box header-right")

    def compose(self):
        yield self.left()
        yield self.center()
        yield self.right()
