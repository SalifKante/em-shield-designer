<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="ru_RU">
<context>
    <name>AssemblyCanvas</name>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="762"/>
        <source>Zoom: 100%</source>
        <translation>Масштаб: 100%</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="878"/>
        <source>Zoom: %1%</source>
        <translation>Масштаб: %1%</translation>
    </message>
</context>
<context>
    <name>BuilderPropertyPanel</name>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1013"/>
        <source>PROPERTIES</source>
        <translation>СВОЙСТВА</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1017"/>
        <source>No element selected</source>
        <translation>Элемент не выбран</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1029"/>
        <source>  CORRECT CIRCUIT:

  1. [Source]
       a, b, t_wall,
       f_start, f_end
     ↓  (series, 0→1)
  2. [Aperture]
       l_slot, w_slot
     ↓  (SHUNT 1→0)
  3. [Cavity]  L
       e.g. 150 mm
       optional internal
       obs (offset = p)
     ↓  (series 1→2)
     … repeat Aperture
       + Cavity pairs …
  4. [Obs.Pt]  (last)
     ↓  (SHUNT, last)
  ╚═ Back-wall short
     auto-added at end

  A Cavity with internal
  observation splits into
  TL_p + TL_(L−p) with an
  obs node between them.

  Use Arrange after
  dropping elements.

  Click element to
  edit its params.</source>
        <translation>  ПРАВИЛЬНАЯ СХЕМА:

  1. [Источник]
       a, b, t_wall,
       f_start, f_end
     ↓  (послед., 0→1)
  2. [Апертура]
       l_slot, w_slot
     ↓  (ШУНТ 1→0)
  3. [Полость]  L
       напр. 150 мм
       опц. внутр.
       набл. (offset = p)
     ↓  (послед. 1→2)
     … повторять пары
       Апертура + Полость …
  4. [Т.набл.]  (последняя)
     ↓  (ШУНТ, последний)
  ╚═ КЗ задней стенки
     добавляется авто

  Полость с внутр.
  наблюдением делится на
  TL_p + TL_(L−p) с узлом
  наблюдения между ними.

  «Упорядочить» после
  добавления элементов.

  Щёлкните элемент для
  правки параметров.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1093"/>
        <source>Label:</source>
        <translation>Метка:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1098"/>
        <source>E₀ [V/m]:</source>
        <translation>E₀ [В/м]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1101"/>
        <source>f start [GHz]:</source>
        <translation>f нач. [ГГц]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1103"/>
        <source>f end [GHz]:</source>
        <translation>f кон. [ГГц]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1105"/>
        <source>Points:</source>
        <translation>Точек:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1107"/>
        <source>Cross-section (shared by all):</source>
        <translation>Поперечное сечение (общее):</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1108"/>
        <source>a [mm]:</source>
        <translation>a [мм]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1110"/>
        <source>b [mm]:</source>
        <translation>b [мм]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1112"/>
        <source>t_wall [mm]:</source>
        <translation>t_wall [мм]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1116"/>
        <location filename="../CircuitBuilderWindow.h" line="1122"/>
        <source>l_slot [mm]:</source>
        <translation>l_slot [мм]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1118"/>
        <location filename="../CircuitBuilderWindow.h" line="1124"/>
        <source>w_slot [mm]:</source>
        <translation>w_slot [мм]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1126"/>
        <source>τ gap [mm]:</source>
        <translation>зазор τ [мм]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1130"/>
        <location filename="../CircuitBuilderWindow.h" line="1142"/>
        <source>L [mm]:</source>
        <translation>L [мм]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1133"/>
        <location filename="../CircuitBuilderWindow.h" line="1149"/>
        <source>Has internal observation</source>
        <translation>Внутренняя точка наблюдения</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1135"/>
        <location filename="../CircuitBuilderWindow.h" line="1151"/>
        <source>obs offset [mm]:</source>
        <translation>смещение набл. [мм]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1144"/>
        <source>h_diel [mm]:</source>
        <translation>h_diel [мм]:</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1158"/>
        <source>SHUNT observation tap.
SE at this node:
SE=-20·log₁₀|2U/V₀|

Z_L &gt;&gt; Z₀ = non-loading.
Default: 1e9 Ω (correct).
WARNING: 377Ω = matched
load → kills resonances!

CORRECT CIRCUIT ORDER:
Source → Aperture →
Cavity → … → Obs.Pt
(Z=1e9, must be last)
Back-wall short added
automatically.

For an obs point inside
a cavity, enable that
cavity&apos;s internal
observation offset.</source>
        <translation>ШУНТ — точка наблюдения.
SE в этом узле:
SE=-20·log₁₀|2U/V₀|

Z_L &gt;&gt; Z₀ = без нагрузки.
По умолч.: 1e9 Ом (верно).
ВНИМАНИЕ: 377Ом = согл.
нагрузка → гасит резонансы!

ПРАВИЛЬНЫЙ ПОРЯДОК:
Источник → Апертура →
Полость → … → Т.набл.
(Z=1e9, должна быть последней)
КЗ задней стенки
добавляется авто.

Для точки наблюдения внутри
полости включите внутреннее
смещение наблюдения полости.</translation>
    </message>
</context>
<context>
    <name>CircuitBuilderWindow</name>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1414"/>
        <source>Circuit Builder — EMShieldDesigner</source>
        <translation>Конструктор схем — EMShieldDesigner</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1533"/>
        <source>ELEMENTS</source>
        <translation>ЭЛЕМЕНТЫ</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1539"/>
        <source>Source</source>
        <translation>Источник</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1540"/>
        <source>Aperture</source>
        <translation>Апертура</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1541"/>
        <source>AP+Cover</source>
        <translation>АП+Крышка</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1542"/>
        <source>Cavity</source>
        <translation>Полость</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1543"/>
        <source>Diel.Cav</source>
        <translation>Диэл.пол.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1544"/>
        <source>Obs.Pt</source>
        <translation>Т.набл.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1560"/>
        <source>ACTIONS</source>
        <translation>ДЕЙСТВИЯ</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1564"/>
        <source>Arrange</source>
        <translation>Упорядочить</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1565"/>
        <source>Delete</source>
        <translation>Удалить</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1566"/>
        <source>Clear</source>
        <translation>Очистить</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1611"/>
        <location filename="../CircuitBuilderWindow.h" line="1985"/>
        <source>Empty canvas</source>
        <translation>Пустой холст</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1626"/>
        <source>COMPUTE</source>
        <translation>РАССЧИТАТЬ</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1632"/>
        <location filename="../CircuitBuilderWindow.h" line="2535"/>
        <source>Export CSV</source>
        <translation>Экспорт CSV</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1714"/>
        <source>Ready  —  correct order: [Source]→[Aperture]→[Cavity(p)]→[Obs.Pt]→[Cavity(d-p)]  |  Last Cavity auto-terminates to ground  |  Compute</source>
        <translation>Готово  —  правильный порядок: [Источник]→[Апертура]→[Полость(p)]→[Т.набл.]→[Полость(d-p)]  |  Последняя полость авто-замыкается на землю  |  Рассчитать</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1730"/>
        <source>Circuit Builder — Shielding Effectiveness</source>
        <translation>Конструктор схем — Эффективность экранирования</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1734"/>
        <source>Frequency [GHz]</source>
        <translation>Частота [ГГц]</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1735"/>
        <source>SE [dB]</source>
        <translation>SE [дБ]</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1776"/>
        <source>Canvas cleared — drag elements to build a new circuit.</source>
        <translation>Холст очищен — перетащите элементы для построения новой схемы.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1790"/>
        <source>Added: %1  |  Arrange left→right: Source → Aperture → Cavity → Obs.Pt</source>
        <translation>Добавлено: %1  |  Упорядочить слева→направо: Источник → Апертура → Полость → Т.набл.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1984"/>
        <source>Circuit valid</source>
        <translation>Схема корректна</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1986"/>
        <source>Source missing</source>
        <translation>Нет источника</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1987"/>
        <source>Multiple Sources</source>
        <translation>Несколько источников</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1988"/>
        <source>Source must be first</source>
        <translation>Источник должен быть первым</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1989"/>
        <source>Obs.Pt missing</source>
        <translation>Нет точки наблюдения</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1990"/>
        <source>Multiple Obs.Pts</source>
        <translation>Несколько точек наблюдения</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1991"/>
        <source>Obs.Pt must be last</source>
        <translation>Точка наблюдения должна быть последней</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1992"/>
        <source>Aperture missing</source>
        <translation>Нет апертуры</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1993"/>
        <source>Cavity missing</source>
        <translation>Нет полости</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1994"/>
        <source>Unbalanced sections</source>
        <translation>Несбалансированные секции</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1995"/>
        <source>Invalid order</source>
        <translation>Неверный порядок</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="1996"/>
        <source>Obs offset out of range</source>
        <translation>Смещение набл. вне диапазона</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2005"/>
        <source>Circuit topology is valid.</source>
        <translation>Топология схемы корректна.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2007"/>
        <source>The canvas is empty.

Drop elements onto the canvas to build a circuit. Minimum legal circuit is:
    Source -&gt; Aperture -&gt; Cavity -&gt; Obs.Pt</source>
        <translation>Холст пуст.

Перетащите элементы на холст для построения схемы. Минимальная допустимая схема:
    Источник -&gt; Апертура -&gt; Полость -&gt; Т.набл.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2013"/>
        <source>No Source element found.

Every circuit needs exactly one Source to provide the excitation voltage V0. Drag a Source element onto the canvas.</source>
        <translation>Источник не найден.

Каждой схеме нужен ровно один источник, задающий возбуждающее напряжение V0. Перетащите элемент «Источник» на холст.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2018"/>
        <source>Multiple Source elements found.

Only one Source is allowed per circuit. Delete the extras so a single excitation V0 drives the chain.</source>
        <translation>Найдено несколько источников.

Допускается только один источник на схему. Удалите лишние, чтобы цепь питало единственное возбуждение V0.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2023"/>
        <source>Source is not the leftmost element.

The Source must be placed at the leftmost X position because the equivalent circuit is read left-to-right starting from V0. Move it to the left of all other elements, or click Arrange.</source>
        <translation>Источник не является крайним слева.

Источник должен располагаться в крайней левой позиции по X, так как эквивалентная схема читается слева направо, начиная с V0. Переместите его левее остальных элементов или нажмите «Упорядочить».</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2029"/>
        <source>No Obs.Pt element found.

Every circuit needs exactly one Obs.Pt where the shielding effectiveness SE = -20 log10|2 U2 / V0| is measured. Drag an Obs.Pt element onto the canvas.</source>
        <translation>Элемент «Т.набл.» не найден.

Каждой схеме нужна ровно одна точка наблюдения, где измеряется эффективность экранирования SE = -20 log10|2 U2 / V0|. Перетащите элемент «Т.набл.» на холст.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2035"/>
        <source>Multiple Obs.Pt elements found.

Only one Obs.Pt is allowed per circuit. Delete the extras.</source>
        <translation>Найдено несколько точек наблюдения.

Допускается только одна точка наблюдения на схему. Удалите лишние.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2039"/>
        <source>Obs.Pt is not the rightmost element.

The Obs.Pt must be placed at the rightmost X position so the back-wall short-circuit termination can be appended after it. Move it to the right of all other elements, or click Arrange.</source>
        <translation>«Т.набл.» не является крайним справа.

Точка наблюдения должна располагаться в крайней правой позиции по X, чтобы после неё можно было добавить короткое замыкание задней стенки. Переместите её правее остальных элементов или нажмите «Упорядочить».</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2045"/>
        <source>No Aperture element found.

The circuit needs at least one Aperture (or AP+Cover) - the coupling element from the external field through the front wall into the cavity.</source>
        <translation>Элемент «Апертура» не найден.

Схеме нужна хотя бы одна апертура (или АП+Крышка) — элемент связи внешнего поля через переднюю стенку с полостью.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2051"/>
        <source>No Cavity element found.

The circuit needs at least one Cavity (or Diel.Cav) section behind the aperture to define the waveguide region of the enclosure interior.</source>
        <translation>Элемент «Полость» не найден.

Схеме нужна хотя бы одна секция «Полость» (или Диэл.пол.) за апертурой, задающая волноводную область внутри корпуса.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2057"/>
        <source>Each Aperture must be paired with a Cavity.

The strict-alternation rule requires the same number of Apertures and Cavities (one per section). Currently they do not match - add or remove elements until the counts are equal.</source>
        <translation>Каждая апертура должна быть в паре с полостью.

Правило строгого чередования требует одинакового числа апертур и полостей (по одной на секцию). Сейчас они не совпадают — добавьте или удалите элементы, пока их количество не сравняется.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2064"/>
        <source>Element order is invalid.

After the Source, the chain must alternate Aperture -&gt; Cavity -&gt; Aperture -&gt; Cavity -&gt; ... and end on a Cavity just before the Obs.Pt. Reorder the elements (drag, or click Arrange) so the pattern is followed.</source>
        <translation>Неверный порядок элементов.

После источника цепь должна чередоваться: Апертура -&gt; Полость -&gt; Апертура -&gt; Полость -&gt; ... и заканчиваться полостью прямо перед «Т.набл.». Измените порядок элементов (перетаскиванием или кнопкой «Упорядочить»), чтобы соблюсти шаблон.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2071"/>
        <source>Cavity internal observation offset is out of range.

When a Cavity has &quot;Has internal observation&quot; enabled, the offset must lie strictly inside the cavity: greater than 0 and less than the cavity length L. Adjust the obs offset, or the cavity length, so that 0 &lt; offset &lt; L.</source>
        <translation>Смещение внутренней точки наблюдения полости вне диапазона.

Когда у полости включена «Внутренняя точка наблюдения», смещение должно лежать строго внутри полости: больше 0 и меньше длины полости L. Измените смещение наблюдения или длину полости так, чтобы 0 &lt; смещение &lt; L.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2154"/>
        <source>Circuit topology error</source>
        <translation>Ошибка топологии схемы</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2156"/>
        <source>Cannot compute — %1.</source>
        <translation>Невозможно рассчитать — %1.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2289"/>
        <source>No observation nodes built (internal error).</source>
        <translation>Узлы наблюдения не построены (внутренняя ошибка).</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2362"/>
        <source>OK  %1 pts · %2 curve(s) · SE: %3…%4 dB · %5–%6 GHz</source>
        <translation>OK  %1 точек · %2 кривых · SE: %3…%4 дБ · %5–%6 ГГц</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2487"/>
        <source>f = %1 GHz
</source>
        <translation>f = %1 ГГц
</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2492"/>
        <source>%1 : %2 dB
</source>
        <translation>%1 : %2 дБ
</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2493"/>
        <source>%1 :   ∞ dB
</source>
        <translation>%1 :   ∞ дБ
</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2527"/>
        <source>No data to export</source>
        <translation>Нет данных для экспорта</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2528"/>
        <source>There is no computed SE data to export.

Click COMPUTE first to populate the plot, then use Export CSV to save the results.</source>
        <translation>Нет рассчитанных данных SE для экспорта.

Сначала нажмите РАССЧИТАТЬ, чтобы построить график, затем используйте «Экспорт CSV» для сохранения результатов.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2535"/>
        <source>CSV (*.csv)</source>
        <translation>Файлы CSV (*.csv)</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2542"/>
        <source>Cannot write file</source>
        <translation>Не удалось записать файл</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2543"/>
        <source>The selected file could not be opened for writing:

%1

Check that the destination folder exists and that the file is not currently open in another program.</source>
        <translation>Не удалось открыть выбранный файл для записи:

%1

Убедитесь, что папка назначения существует и файл не открыт в другой программе.</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2559"/>
        <source>Exported: </source>
        <translation>Экспортировано: </translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2567"/>
        <source>Export complete</source>
        <translation>Экспорт завершён</translation>
    </message>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="2568"/>
        <source>CSV saved successfully to:

%1</source>
        <translation>CSV успешно сохранён в:

%1</translation>
    </message>
</context>
<context>
    <name>MainWindow</name>
    <message>
        <location filename="../mainwindow.cpp" line="65"/>
        <source>Quick Simulation — EMShieldDesigner</source>
        <translation>Быстрое моделирование — EMShieldDesigner</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="284"/>
        <source>PRESETS</source>
        <translation>ПРЕСЕТЫ</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="287"/>
        <source>Config:</source>
        <translation>Конфиг.:</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="288"/>
        <source>1-Section (baseline)</source>
        <translation>1 секция (базовая)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="289"/>
        <source>2-Section identical</source>
        <translation>2 секции (одинаковые)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="290"/>
        <source>3-Section identical</source>
        <translation>3 секции (одинаковые)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="291"/>
        <source>2-Section different</source>
        <translation>2 секции (разные)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="292"/>
        <source>5-Section cascade</source>
        <translation>5 секций (каскад)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="293"/>
        <source>Custom (edit below)</source>
        <translation>Произвольно (правка ниже)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="305"/>
        <source>ENCLOSURE</source>
        <translation>КОРПУС</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="308"/>
        <source>a [mm]:</source>
        <translation>a [мм]:</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="310"/>
        <source>b [mm]:</source>
        <translation>b [мм]:</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="312"/>
        <source>t [mm]:</source>
        <translation>t [мм]:</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="316"/>
        <source>Topology:</source>
        <translation>Топология:</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="317"/>
        <source>Cascade</source>
        <translation>Каскад</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="317"/>
        <source>Star-branch</source>
        <translation>Звезда-ветвь</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="319"/>
        <source>CASCADE:     sections connected serially in depth.
STAR_BRANCH: section 1 is the spine; sections 2..N branch
             laterally from the spine output junction.</source>
        <translation>КАСКАД:     секции соединены последовательно по глубине.
ЗВЕЗДА-ВЕТВЬ: секция 1 — ствол; секции 2..N ответвляются
             от выходного узла ствола.</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="327"/>
        <source>FREQUENCY SWEEP</source>
        <translation>ЧАСТОТНАЯ РАЗВЁРТКА</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="329"/>
        <source>Start:</source>
        <translation>Начало:</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="331"/>
        <source>Stop:</source>
        <translation>Конец:</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="333"/>
        <source>Points:</source>
        <translation>Точек:</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="336"/>
        <source>SECTION PROPERTIES</source>
        <translation>СВОЙСТВА СЕКЦИИ</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="342"/>
        <source>ACTIONS</source>
        <translation>ДЕЙСТВИЯ</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="344"/>
        <source>Add Section</source>
        <translation>Добавить секцию</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="349"/>
        <source>Add a new section (Ctrl+N)</source>
        <translation>Добавить новую секцию (Ctrl+N)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="353"/>
        <source>Remove Section</source>
        <translation>Удалить секцию</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="358"/>
        <source>Remove selected section (Delete)</source>
        <translation>Удалить выбранную секцию (Delete)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="392"/>
        <source>Checking...</source>
        <translation>Проверка...</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="397"/>
        <source>COMPUTE</source>
        <translation>РАССЧИТАТЬ</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="403"/>
        <source>Run shielding analysis (Ctrl+R)</source>
        <translation>Запустить анализ экранирования (Ctrl+R)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="407"/>
        <location filename="../mainwindow.cpp" line="815"/>
        <source>Export CSV</source>
        <translation>Экспорт CSV</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="412"/>
        <source>Export results to CSV (Ctrl+S)</source>
        <translation>Экспортировать результаты в CSV (Ctrl+S)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="443"/>
        <source>Quick Simulation — Shielding Effectiveness</source>
        <translation>Быстрое моделирование — Эффективность экранирования</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="449"/>
        <source>Frequency [GHz]</source>
        <translation>Частота [ГГц]</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="450"/>
        <source>SE [dB]</source>
        <translation>SE [дБ]</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="498"/>
        <source>Ready</source>
        <translation>Готово</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="605"/>
        <source>Bad frequency span</source>
        <translation>Неверный частотный диапазон</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="606"/>
        <source>Frequency stop must be greater than frequency start.

Increase the Stop value or decrease the Start value.</source>
        <translation>Конечная частота должна быть больше начальной.

Увеличьте значение «Конец» или уменьшите «Начало».</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="612"/>
        <source>Too few points</source>
        <translation>Слишком мало точек</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="613"/>
        <source>Frequency sweep needs at least 2 points so both endpoints are included in the linspace.</source>
        <translation>Частотной развёртке нужно не менее 2 точек, чтобы обе граничные точки вошли в диапазон.</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="639"/>
        <source>Configuration is not valid for analysis:

%1

Fix the highlighted condition, then COMPUTE will be safe to run.</source>
        <translation>Конфигурация недопустима для анализа:

%1

Исправьте указанное условие, после чего РАССЧИТАТЬ можно будет запустить безопасно.</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="648"/>
        <source>Circuit valid</source>
        <translation>Схема корректна</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="649"/>
        <source>Configuration is valid. Click COMPUTE to run the sweep.</source>
        <translation>Конфигурация корректна. Нажмите РАССЧИТАТЬ для запуска развёртки.</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="743"/>
        <source>Added section %1 — click Compute to update</source>
        <translation>Добавлена секция %1 — нажмите «Рассчитать» для обновления</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="751"/>
        <source>Cannot remove the last section</source>
        <translation>Нельзя удалить последнюю секцию</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="760"/>
        <source>%1 sections remaining — click Compute to update</source>
        <translation>Осталось секций: %1 — нажмите «Рассчитать» для обновления</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="806"/>
        <source>No data to export</source>
        <translation>Нет данных для экспорта</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="807"/>
        <source>There is no computed SE data to export.

Click COMPUTE first to populate the plot, then use Export CSV to save the results.</source>
        <translation>Нет рассчитанных данных SE для экспорта.

Сначала нажмите РАССЧИТАТЬ, чтобы построить график, затем используйте «Экспорт CSV» для сохранения результатов.</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="817"/>
        <source>CSV Files (*.csv)</source>
        <translation>Файлы CSV (*.csv)</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="823"/>
        <source>Cannot write file</source>
        <translation>Не удалось записать файл</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="824"/>
        <source>The selected file could not be opened for writing:

%1

Check that the destination folder exists and that the file is not currently open in another program.</source>
        <translation>Не удалось открыть выбранный файл для записи:

%1

Убедитесь, что папка назначения существует и файл не открыт в другой программе.</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="847"/>
        <source>Exported: </source>
        <translation>Экспортировано: </translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="850"/>
        <source>Export complete</source>
        <translation>Экспорт завершён</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="851"/>
        <source>CSV saved successfully to:

%1</source>
        <translation>CSV успешно сохранён в:

%1</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="896"/>
        <source>No sections</source>
        <translation>Нет секций</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="897"/>
        <source>The canvas has no sections.

Use Add Section or load a preset, then click COMPUTE.</source>
        <translation>На холсте нет секций.

Используйте «Добавить секцию» или загрузите пресет, затем нажмите РАССЧИТАТЬ.</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="899"/>
        <source>No sections defined</source>
        <translation>Секции не заданы</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="926"/>
        <source>Invalid configuration</source>
        <translation>Недопустимая конфигурация</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="928"/>
        <source>Cannot compute — invalid configuration</source>
        <translation>Невозможно рассчитать — недопустимая конфигурация</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="932"/>
        <source>Computing...</source>
        <translation>Расчёт...</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="942"/>
        <location filename="../mainwindow.cpp" line="944"/>
        <source>Circuit generation failed</source>
        <translation>Сбой генерации схемы</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="950"/>
        <location filename="../mainwindow.cpp" line="955"/>
        <source>No observation points</source>
        <translation>Нет точек наблюдения</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="951"/>
        <source>No sections have &apos;Has observation point&apos; enabled.

Enable at least one section&apos;s observation in the SECTION PROPERTIES panel, then COMPUTE again.</source>
        <translation>Ни в одной секции не включена «Точка наблюдения».

Включите наблюдение хотя бы в одной секции на панели «СВОЙСТВА СЕКЦИИ», затем снова нажмите РАССЧИТАТЬ.</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="1002"/>
        <source>%1-section %2 | %3 branches, %4 nodes, %5 obs pts | %6 points in %7 ms</source>
        <translation>%1-секц. %2 | ветвей: %3, узлов: %4, т.набл.: %5 | %6 точек за %7 мс</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="1162"/>
        <source>f = %1 GHz
</source>
        <translation>f = %1 ГГц
</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="1168"/>
        <source>%1 : %2 dB
</source>
        <translation>%1 : %2 дБ
</translation>
    </message>
    <message>
        <location filename="../mainwindow.cpp" line="1170"/>
        <source>%1 :   inf dB
</source>
        <translation>%1 :   inf дБ
</translation>
    </message>
</context>
<context>
    <name>MessageDialog</name>
    <message>
        <location filename="../MessageDialog.h" line="178"/>
        <source>Close</source>
        <translation>Закрыть</translation>
    </message>
    <message>
        <location filename="../MessageDialog.h" line="179"/>
        <source>OK</source>
        <translation>OK</translation>
    </message>
</context>
<context>
    <name>PaletteButton</name>
    <message>
        <location filename="../CircuitBuilderWindow.h" line="447"/>
        <source>Drag %1 onto canvas</source>
        <translation>Перетащите %1 на холст</translation>
    </message>
</context>
<context>
    <name>PropertyPanel</name>
    <message>
        <location filename="../PropertyPanel.h" line="63"/>
        <source>Section %1 Properties</source>
        <translation>Свойства секции %1</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="101"/>
        <location filename="../PropertyPanel.h" line="122"/>
        <source>No Section Selected</source>
        <translation>Секция не выбрана</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="128"/>
        <source>Cavity</source>
        <translation>Полость</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="145"/>
        <source>Depth:</source>
        <translation>Глубина:</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="151"/>
        <source>Obs position:</source>
        <translation>Позиция набл.:</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="154"/>
        <source>Has observation point</source>
        <translation>Есть точка наблюдения</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="163"/>
        <source>Width Override (STAR_BRANCH)</source>
        <translation>Переопр. ширины (STAR_BRANCH)</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="167"/>
        <source>Set &gt; 0 to override the global enclosure
width for this section.
0 = use global a.</source>
        <translation>Установите &gt; 0, чтобы переопределить общую ширину
корпуса для этой секции.
0 = использовать общую a.</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="175"/>
        <source>Width (a):</source>
        <translation>Ширина (a):</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="182"/>
        <source>Aperture</source>
        <translation>Апертура</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="186"/>
        <source>Width (l):</source>
        <translation>Ширина (l):</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="188"/>
        <source>Height (w):</source>
        <translation>Высота (w):</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="195"/>
        <source>Aperture Cover</source>
        <translation>Крышка апертуры</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="199"/>
        <source>Enable cover</source>
        <translation>Включить крышку</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="202"/>
        <source>Gap (τ):</source>
        <translation>Зазор (τ):</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="207"/>
        <source>Gap εr:</source>
        <translation>Зазор εr:</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="210"/>
        <source>Relative permittivity of the cover gap filler.
1.0 = air gap.
&gt; 1.0 = dielectric filler.</source>
        <translation>Относительная диэлектрическая проницаемость наполнителя зазора крышки.
1.0 = воздушный зазор.
&gt; 1.0 = диэлектрический наполнитель.</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="216"/>
        <location filename="../PropertyPanel.h" line="332"/>
        <source>Air gap</source>
        <translation>Воздушный зазор</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="225"/>
        <source>Dielectric Fill</source>
        <translation>Диэлектрическое заполнение</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="229"/>
        <source>Enable dielectric</source>
        <translation>Включить диэлектрик</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="232"/>
        <source>Height (h):</source>
        <translation>Высота (h):</translation>
    </message>
    <message>
        <location filename="../PropertyPanel.h" line="330"/>
        <source>Dielectric (εr = %1)</source>
        <translation>Диэлектрик (εr = %1)</translation>
    </message>
</context>
<context>
    <name>SectionItem</name>
    <message>
        <location filename="../SectionItem.h" line="177"/>
        <source>Ap: %1×%2</source>
        <translation>Ап: %1×%2</translation>
    </message>
    <message>
        <location filename="../SectionItem.h" line="243"/>
        <source>Section %1
Depth:    %2 mm
Obs pos:  %3 mm
Aperture: %4 × %5 mm</source>
        <translation>Секция %1
Глубина:  %2 мм
Поз.набл: %3 мм
Апертура: %4 × %5 мм</translation>
    </message>
    <message>
        <location filename="../SectionItem.h" line="255"/>
        <source>Cover gap: %1 mm</source>
        <translation>Зазор крышки: %1 мм</translation>
    </message>
    <message>
        <location filename="../SectionItem.h" line="258"/>
        <source>εr=%1 (dielectric)</source>
        <translation>εr=%1 (диэлектрик)</translation>
    </message>
    <message>
        <location filename="../SectionItem.h" line="261"/>
        <source>(air)</source>
        <translation>(воздух)</translation>
    </message>
    <message>
        <location filename="../SectionItem.h" line="264"/>
        <source>Dielectric: εr=%1, h=%2 mm</source>
        <translation>Диэлектрик: εr=%1, h=%2 мм</translation>
    </message>
    <message>
        <location filename="../SectionItem.h" line="268"/>
        <source>Width override: a=%1 mm</source>
        <translation>Переопр. ширины: a=%1 мм</translation>
    </message>
</context>
<context>
    <name>StackLayerCard</name>
    <message>
        <location filename="../StackLayerPanel.h" line="314"/>
        <source>SOURCE</source>
        <translation>ИСТОЧНИК</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="315"/>
        <source>APERTURE</source>
        <translation>АПЕРТУРА</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="316"/>
        <source>AP+COVER</source>
        <translation>АП+КРЫШ</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="317"/>
        <source>CAVITY</source>
        <translation>ПОЛОСТЬ</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="318"/>
        <source>DIEL.CAV</source>
        <translation>ДИЭЛ.ПОЛ</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="319"/>
        <source>OBS.PT</source>
        <translation>Т.НАБЛ</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="321"/>
        <source>ELEMENT</source>
        <translation>ЭЛЕМЕНТ</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="347"/>
        <source>E₀ = %1 V/m
f: %2 – %3 GHz   (%4 pts)
a×b = %5×%6 mm
t_wall = %7 mm</source>
        <translation>E₀ = %1 В/м
f: %2 – %3 ГГц   (%4 тчк)
a×b = %5×%6 мм
t_wall = %7 мм</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="362"/>
        <source>l×w = %1×%2 mm</source>
        <translation>l×w = %1×%2 мм</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="367"/>
        <source>l×w = %1×%2 mm
τ gap = %3 mm</source>
        <translation>l×w = %1×%2 мм
зазор τ = %3 мм</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="376"/>
        <source>L = %1 mm</source>
        <translation>L = %1 мм</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="378"/>
        <location filename="../StackLayerPanel.h" line="391"/>
        <source>obs @ %1 mm</source>
        <translation>набл. @ %1 мм</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="382"/>
        <source>L = %1 mm
h_diel = %2 mm
εᵣ = %3</source>
        <translation>L = %1 мм
h_diel = %2 мм
εᵣ = %3</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="398"/>
        <source>Z_L = %1 Ω</source>
        <translation>Z_L = %1 Ом</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="400"/>
        <source>Z_L = %1 %2 j%3 Ω</source>
        <translation>Z_L = %1 %2 j%3 Ом</translation>
    </message>
</context>
<context>
    <name>StackLayerPanel</name>
    <message>
        <location filename="../StackLayerPanel.h" line="427"/>
        <source>EQUIVALENT CIRCUIT LAYERS</source>
        <translation>СЛОИ ЭКВИВАЛЕНТНОЙ СХЕМЫ</translation>
    </message>
    <message>
        <location filename="../StackLayerPanel.h" line="473"/>
        <source>No elements yet.

Drop elements onto the canvas
to see them stack here.</source>
        <translation>Пока нет элементов.

Перетащите элементы на холст,
чтобы увидеть их стек здесь.</translation>
    </message>
</context>
<context>
    <name>StartupWindow</name>
    <message>
        <location filename="../StartupWindow.h" line="271"/>
        <source>Shielding Effectiveness Analyzer
Using Equivalent Circuit Method and Nodal Analysis</source>
        <translation>Анализатор эффективности экранирования
Метод эквивалентных схем и узловых потенциалов</translation>
    </message>
    <message>
        <location filename="../StartupWindow.h" line="300"/>
        <source>Quick Simulation</source>
        <translation>Быстрое моделирование</translation>
    </message>
    <message>
        <location filename="../StartupWindow.h" line="301"/>
        <source>Preset configurations with
interactive parameter control</source>
        <translation>Готовые конфигурации с
интерактивным управлением параметрами</translation>
    </message>
    <message>
        <location filename="../StartupWindow.h" line="308"/>
        <source>Circuit Builder</source>
        <translation>Конструктор схем</translation>
    </message>
    <message>
        <location filename="../StartupWindow.h" line="309"/>
        <source>Drag &amp; drop elements to build
custom equivalent circuits</source>
        <translation>Перетаскивайте элементы для построения
произвольных эквивалентных схем</translation>
    </message>
    <message>
        <location filename="../StartupWindow.h" line="320"/>
        <source>Electromagnetic Compatibility Research Tool</source>
        <translation>Инструмент исследования электромагнитной совместимости</translation>
    </message>
</context>
</TS>
