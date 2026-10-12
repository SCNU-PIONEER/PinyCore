"""Run menuconfig with LK communication choices expanded under each model."""

from kconfiglib import Symbol, standard_kconfig
import menuconfig as ui


changeNode = ui._change_node


def changeInlineChoice(_node):
    symbol = _node.item
    if isinstance(symbol, Symbol) and symbol.choice and not _node.parent.is_menuconfig:
        # Keep the model page open after selecting an inline radio button.
        ui._set_val(symbol, 2)
        return True
    return changeNode(_node)


def expandLkCommunication(_kconf):
    for name in (
        "LK_MF9025_COMMUNICATION",
        "LK_MG4005_COMMUNICATION",
        "LK_MG8016EI6_COMMUNICATION",
    ):
        for node in _kconf.named_choices[name].nodes:
            node.is_menuconfig = False


if __name__ == "__main__":
    kconf = standard_kconfig()
    expandLkCommunication(kconf)
    ui._change_node = changeInlineChoice
    ui.menuconfig(kconf)
