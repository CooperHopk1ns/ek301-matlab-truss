%% EK301 TRUSS DESIGN PROJECT - PRELIMINARY DESIGN ANALYSIS (Fall 2026)
% =========================================================================
% File    : TrussPrelimDesign.m
% Needs   : MATLAB R2016b or newer (local functions at the end of a script).
%           Only base MATLAB is used (no toolboxes).
%
% WHAT THIS FILE DOES (maps to the Preliminary Design Manual)
%   Section 0  Settings           : group info, buckling fit (Eq. 10), cost (Eq. 14), specs
%   Section 1  Model verification : solves the practice truss and compares to the hand solution
%   Section 2  Define designs     : two candidate trusses (STARTER DESIGNS - replace with yours)
%   Section 3  Analyze designs    : spec checks, member forces, critical member, max load,
%                                   cost, load/cost ratio, diagram, results table
%   Section 4  Compare designs    : side-by-side summary for the Discussion section
%   Section 5  Design file        : saves the required .mat file and re-loads it as a test
%
% SIGN CONVENTION: member force > 0 = TENSION, < 0 = COMPRESSION.
% UNITS          : designs use inches and oz. The practice truss uses m and N (no buckling).
%
% REUSING THE FUNCTIONS: every local function below (after Section 5) is
% self-contained. Copy one into its own file named <functionname>.m to use
% it from any other script.
% =========================================================================
clear; clc; close all;

%% ======================= SECTION 0: SETTINGS =============================
% Group information (printed in the required output header)
info.section = 'A1';
info.group   = 'Group 1';
info.names   = 'First Last, First Last, First Last';
info.fileTag = 'FirstLastFirstLast';          % used in the .mat file name
info.date    = datestr(now, 'mm/dd/yyyy');

% Buckling fit for Fall 2026 (Eq. 10):  Fb = Cfit * L^(-alpha) +/- Ufit
bk.Cfit  = 3391.3;     % oz
bk.alpha = 2.012;      % -
bk.Ufit  = 1.52;       % oz (total uncertainty of the fit)

% Cost model (Eq. 14). The manual writes the currency as "$301" (EK301 dollars),
% so this is read as: $1 per inch of member + $10 per joint.
% >>> Confirm with your GST/instructor; change these two numbers if different. <<<
costModel.perInch  = 1;
costModel.perJoint = 10;

% Design specifications (Section 1.2 of the manual)
spec.Wmin       = 30;          % oz, minimum load to carry for 60 s
spec.spanRange  = [30 33];     % in, pin-to-roller span
spec.dyRange    = [0.5 1.5];   % in, load joint height above pin joint
spec.LjjRange   = [8.5 14];    % in, joint-to-joint length (applied to EVERY member)
spec.checkLjj   = true;        % set false if "Ljj" is not a per-member limit
spec.barLengths = [48 48 24];  % in, available stock: two 4-ft bars + one 2-ft bar

Wtest = 30;                    % oz, load used for the member-force printout

%% ======================= SECTION 1: MODEL VERIFICATION ===================
% Practice truss from the worked-out example (25 N down at joint D).
% Joints: A=1 B=2 C=3 D=4 E=5 F=6 G=7 H=8   (coordinates in meters)
vNames   = {'AB','AD','BC','BD','CD','CE','DE','DF','EF','EG','FG','FH','GH'};
vX       = [0 0 4 4 8 8 12 12];
vY       = [0 4 8 4 8 4  4  0];
vMembers = [1 2; 1 4; 2 3; 2 4; 3 4; 3 5; 4 5; 4 6; 5 6; 5 7; 6 7; 6 8; 7 8];

% pin at A (joint 1), roller at H (joint 8), 25 N load at D (joint 4)
V = buildTruss('Practice truss', vX, vY, vMembers, 1, 8, 4, 25);
[Fv, eV] = solveTruss(V.C, V.Sx, V.Sy, V.X, V.Y, V.L);

% Hand solution in the same member order (+ = tension, - = compression), N.
% NOTE: the posted answer sheet lists EG = 11.79 N (T) and FG = 8.33 N (C).
% Joint equilibrium at G, E and F gives the OPPOSITE signs (EG compression,
% FG tension), and the sheet's own method-of-joints work for joint E only
% balances with EG in compression. The values below use the corrected signs.
Fhand = [-16.67; 0; -23.57; 16.67; 16.67; -16.67; 11.79; 8.33; 0; ...
         -11.79; 8.33; 0; -8.33];
reactHand = [0; 16.67; 8.33];     % Ax, Ay, Hy

fprintf('=== MODEL VERIFICATION: practice truss (25 N at joint D) ===\n');
okMem = compareWithHand(vNames, Fhand, Fv(1:V.M), 0.05);
okRea = compareWithHand({'Sx1','Sy1','Sy2'}, reactHand, Fv(V.M+1:end), 0.05);
fprintf('Max equilibrium residual |e*F - L| = %.2e N (should be ~1e-14)\n', ...
        max(abs(eV*Fv - V.L)));
if okMem && okRea
    fprintf('VERIFICATION PASSED: code and hand solution agree.\n\n');
else
    fprintf('VERIFICATION FAILED: resolve the mismatches before submitting.\n\n');
end

%% ======================= SECTION 2: DEFINE CANDIDATE DESIGNS =============
% STARTER DESIGNS - they run and meet the specs, but replace them with your
% own. Each design is just: joint X, joint Y, member list (joint pairs), then
% pin joint, roller joint, load joint, load in oz. Everything else is automatic.
%
% Design 1: flat-top Warren truss, 7 joints / 11 members, load joint 1.0 in up
D1 = buildTruss('Design 1 (flat-top Warren)', ...
        [0 10  18.5 32 5    14.25 25.25], ...
        [0 1.0 0    0  8    9    8    ], ...
        [1 2; 2 3; 3 4; 1 5; 5 2; 5 6; 6 2; 6 3; 6 7; 7 3; 7 4], ...
        1, 4, 2, Wtest);

% Design 2: peaked-top Warren truss, 7 joints / 11 members, load joint 1.5 in up
D2 = buildTruss('Design 2 (peaked-top Warren)', ...
        [0 12.5 21 32 5.5 14.75 24 ], ...
        [0 1.5  0  0  8.5 11    8.5], ...
        [1 2; 2 3; 3 4; 1 5; 5 2; 5 6; 6 2; 6 3; 6 7; 7 3; 7 4], ...
        1, 4, 2, Wtest);

designs = {D1, D2};

%% ======================= SECTION 3: ANALYZE EACH DESIGN ==================
results = cell(size(designs));
for k = 1:numel(designs)
    T = designs{k};
    fprintf('\n=====================  %s  =====================\n', T.name);

    checkDesignSpecs(T, spec);                 % spec compliance
    res = analyzeDesign(T, bk, costModel);     % forces, critical member, Wmax, cost
    if res.Wmax < spec.Wmin
        warning('%s only supports %.1f oz (< %g oz required).', ...
                T.name, res.Wmax, spec.Wmin);
    end

    fprintf('\n--- Required output format (load = %g oz) ---\n', T.W);
    printTrussReport(T, res, info);
    fprintf('\n--- Items 2-4 of Section 2.5 (critical member, max load, cost) ---\n');
    printMaxLoadSummary(T, res);
    fprintf('\n--- Results table (Results section, item 1) ---\n');
    printMemberTable(T, res);

    plotTruss(T, res);
    saveas(gcf, sprintf('Design%d_diagram.png', k));
    writeResultsCSV(T, res, sprintf('Design%d_results.csv', k));

    results{k} = res;
end

%% ======================= SECTION 4: COMPARE DESIGNS ======================
fprintf('\n=====================  DESIGN COMPARISON  =====================\n');
fprintf('%-30s %6s %6s %10s %12s %10s %10s\n', 'Design', 'Joints', 'Memb.', ...
        'Cost ($)', 'Max load(oz)', '+/- (oz)', 'oz per $');
for k = 1:numel(designs)
    T = designs{k}; r = results{k};
    fprintf('%-30s %6d %6d %10.2f %12.2f %10.2f %10.4f\n', T.name, T.J, T.M, ...
            r.cost, r.Wmax, r.Wunc, r.ratio);
end
[~, best] = max(cellfun(@(r) r.ratio, results));
fprintf('Highest load-to-cost ratio: %s\n', designs{best}.name);

%% ======================= SECTION 5: DESIGN FILE FOR GRADESCOPE ===========
% Required: .mat file with C, Sx, Sy, X, Y, L for the FIRST design.
matName = sprintf('TrussDesign1_%s_%s.mat', info.fileTag, info.section);
saveDesignFile(designs{1}, matName);
fprintf('\nSaved design file: %s\n', matName);

% Test: reload the file exactly as the graders will, and re-solve
S  = load(matName);
F2 = solveTruss(S.C, S.Sx, S.Sy, S.X, S.Y, S.L);
fprintf('Reloaded-file check: max |F_file - F_design| = %.2e oz\n', ...
        max(abs(F2 - results{1}.F)));

% =========================================================================
%                          LOCAL FUNCTIONS (reusable)
% =========================================================================

function T = buildTruss(name, X, Y, members, pinJoint, rollerJoint, loadJoint, W)
% BUILDTRUSS  Package a truss definition and build C, Sx, Sy, L automatically.
%   members : M-by-2 list of joint-number pairs (one row per member)
%   W       : magnitude of the downward load at loadJoint
    T.name      = name;
    T.X         = X(:)';              % row vectors, as in the manual
    T.Y         = Y(:)';
    T.members   = members;
    T.J         = numel(X);
    T.M         = size(members, 1);
    T.pin       = pinJoint;
    T.roller    = rollerJoint;
    T.loadJoint = loadJoint;
    T.W         = W;
    T.C         = connectionMatrix(members, T.J);
    [T.Sx, T.Sy] = supportMatrices(T.J, pinJoint, rollerJoint);
    T.L         = loadVector(T.J, loadJoint, 0, -W);   % applied force (0, -W)
end

function C = connectionMatrix(members, J)
% CONNECTIONMATRIX  J-by-M matrix; C(j,m) = 1 if member m touches joint j.
    M = size(members, 1);
    C = zeros(J, M);
    for m = 1:M
        C(members(m,1), m) = 1;
        C(members(m,2), m) = 1;
    end
end

function [Sx, Sy] = supportMatrices(J, pinJoint, rollerJoint)
% SUPPORTMATRICES  J-by-3 matrices marking the unknown reactions.
%   Column order: Sx1 (pin, x), Sy1 (pin, y), Sy2 (roller, y).
    Sx = zeros(J, 3);  Sy = zeros(J, 3);
    Sx(pinJoint, 1)    = 1;
    Sy(pinJoint, 2)    = 1;
    Sy(rollerJoint, 3) = 1;
end

function L = loadVector(J, joint, Fx, Fy)
% LOADVECTOR  2J-by-1 vector L. First J rows = x equations, last J = y.
%   Fx, Fy are the APPLIED force components at 'joint'. Loads are moved to the
%   right-hand side of the equilibrium equations, so the sign flips (Eq. 8).
    L = zeros(2*J, 1);
    L(joint)     = -Fx;
    L(J + joint) = -Fy;
end

function len = memberLengths(C, X, Y)
% MEMBERLENGTHS  M-by-1 vector of member lengths from C and joint coordinates.
    M = size(C, 2);  len = zeros(M, 1);
    for m = 1:M
        j = find(C(:, m));
        len(m) = hypot(X(j(2)) - X(j(1)), Y(j(2)) - Y(j(1)));
    end
end

function [F, eMat] = solveTruss(C, Sx, Sy, X, Y, L)
% SOLVETRUSS  Solve all member forces and support reactions (Section 2.4).
%   Builds the 2J-by-(M+3) matrix eMat from C, Sx, Sy, X, Y, then solves
%   eMat*F = L.   F = [F1..FM; Sx1; Sy1; Sy2]   (+ = tension)
    [J, M] = size(C);
    X = X(:);  Y = Y(:);  L = L(:);
    eMat = zeros(2*J, M + 3);
    for m = 1:M
        j = find(C(:, m));
        if numel(j) ~= 2
            error('Member %d must connect exactly two joints (column sum of C = 2).', m);
        end
        a = j(1);  b = j(2);
        d = [X(b) - X(a); Y(b) - Y(a)];
        u = d / norm(d);                 % unit vector pointing from joint a to joint b
        eMat(a,   m) =  u(1);  eMat(b,   m) = -u(1);     % x-equilibrium rows
        eMat(J+a, m) =  u(2);  eMat(J+b, m) = -u(2);     % y-equilibrium rows
    end
    eMat(1:J,       M+1:M+3) = Sx;       % reaction columns
    eMat(J+1:2*J,   M+1:M+3) = Sy;
    if size(eMat,1) ~= size(eMat,2) || rcond(eMat) < 1e-12
        error(['Equilibrium matrix is not square/invertible. Check M = 2J-3, the ' ...
               'connection matrix, and that Sx1 is included in the supports.']);
    end
    F = eMat \ L;    % same result as inv(eMat)*L (Eq. 9) but more accurate
end

function [Fb, Ub] = bucklingStrength(len, bk)
% BUCKLINGSTRENGTH  Eq. 10:  Fb = Cfit * L^(-alpha), uncertainty Ub = Ufit.
    Fb = bk.Cfit .* len .^ (-bk.alpha);
    Ub = bk.Ufit * ones(size(len));
end

function [cost, Ltot] = trussCost(len, J, costModel)
% TRUSSCOST  Eq. 14: cost = perInch*Ltotal + perJoint*J.
    Ltot = sum(len);
    cost = costModel.perInch * Ltot + costModel.perJoint * J;
end

function res = analyzeDesign(T, bk, costModel)
% ANALYZEDESIGN  Everything Section 2.5 asks for, for one truss.
    % (1) member forces and reactions at the nominal load T.W
    [F, eMat] = solveTruss(T.C, T.Sx, T.Sy, T.X, T.Y, T.L);
    res.F         = F;
    res.Fmem      = F(1:T.M);
    res.reactions = F(T.M+1:end);
    res.resid     = max(abs(eMat*F - T.L));
    res.len       = memberLengths(T.C, T.X, T.Y);

    % (2) buckling strength of each member from its length
    [res.Fb, res.Ub] = bucklingStrength(res.len, bk);

    % (3) scale by linearity: Fm = Rm*W  ->  Rm = Fm/W  (Eq. 11)
    res.R    = res.Fmem / T.W;
    res.comp = res.R < -1e-9;                   % only compression members buckle
    res.Wfail = inf(T.M, 1);
    res.Wfail(res.comp) = res.Fb(res.comp) ./ abs(res.R(res.comp));  % Eq. 12-13

    % (4) critical member = smallest failure load
    [res.Wmax, res.critical] = min(res.Wfail);
    c = res.critical;
    res.Wunc  = bk.Ufit / abs(res.R(c));         % uncertainty from critical member only
    res.Fcrit = res.Fb(c);

    % (5) cost and load-to-cost ratio
    [res.cost, res.Ltot] = trussCost(res.len, T.J, costModel);
    res.ratio = res.Wmax / res.cost;
end

function allOk = checkDesignSpecs(T, spec)
% CHECKDESIGNSPECS  Verify a design against Section 1.2 and print PASS/FAIL.
    tol = 1e-9;
    len = memberLengths(T.C, T.X, T.Y);
    allOk = true;
    fprintf('Specification checks:\n');

    % simple truss bookkeeping (triangles only <=> M = 2J-3 with an invertible e-matrix)
    allOk = reportCheck(allOk, 'Simple truss: M = 2J - 3', T.M == 2*T.J - 3, ...
        sprintf('J = %d, M = %d, 2J-3 = %d', T.J, T.M, 2*T.J - 3));
    allOk = reportCheck(allOk, 'Every joint has >= 2 members', all(sum(T.C, 2) >= 2), ...
        sprintf('min members per joint = %d', min(sum(T.C, 2))));

    % members may not cross
    crossing = false;
    for a = 1:T.M-1
        for b = a+1:T.M
            ja = find(T.C(:, a));  jb = find(T.C(:, b));
            if ~isempty(intersect(ja, jb)), continue; end     % share a joint
            if segmentsCross([T.X(ja(1)) T.Y(ja(1))], [T.X(ja(2)) T.Y(ja(2))], ...
                             [T.X(jb(1)) T.Y(jb(1))], [T.X(jb(2)) T.Y(jb(2))])
                crossing = true;
            end
        end
    end
    allOk = reportCheck(allOk, 'No members cross', ~crossing, '');

    % supports on the bottom axis, nothing below the support line
    allOk = reportCheck(allOk, 'Supports on bottom axis, no joint below it', ...
        abs(T.Y(T.pin) - T.Y(T.roller)) < tol && all(T.Y >= T.Y(T.pin) - tol), ...
        sprintf('min Y = %.2f in', min(T.Y)));

    % span and load-joint height
    span = T.X(T.roller) - T.X(T.pin);
    allOk = reportCheck(allOk, 'Truss span in range', ...
        span >= spec.spanRange(1) - tol && span <= spec.spanRange(2) + tol, ...
        sprintf('%.2f in, allowed %g-%g', span, spec.spanRange));
    dy = T.Y(T.loadJoint) - T.Y(T.pin);
    allOk = reportCheck(allOk, 'Load joint height above pin in range', ...
        dy >= spec.dyRange(1) - tol && dy <= spec.dyRange(2) + tol, ...
        sprintf('%.2f in, allowed %g-%g', dy, spec.dyRange));
    below = (abs(T.X - T.X(T.loadJoint)) < 1e-6) & (T.Y < T.Y(T.loadJoint) - tol);
    allOk = reportCheck(allOk, 'No joint directly below load joint', ~any(below), '');

    % member length range (interpretation of "joint-to-joint span")
    if spec.checkLjj
        allOk = reportCheck(allOk, 'Member lengths in range', ...
            all(len >= spec.LjjRange(1) - tol & len <= spec.LjjRange(2) + tol), ...
            sprintf('%.2f to %.2f in, allowed %g-%g', min(len), max(len), spec.LjjRange));
    end

    % material: total length and whether the members can be cut from the bars
    [fits, assign, remaining] = packBars(len, spec.barLengths);
    allOk = reportCheck(allOk, 'Total length within stock', ...
        sum(len) <= sum(spec.barLengths), ...
        sprintf('%.2f of %g in', sum(len), sum(spec.barLengths)));
    allOk = reportCheck(allOk, 'Members can be cut from the bars', fits, ...
        'first-fit-decreasing, ignores saw-kerf waste');
    if fits
        for b = 1:numel(spec.barLengths)
            fprintf('      bar %d (%g in): members %s, %.2f in left\n', b, ...
                spec.barLengths(b), mat2str(find(assign == b)'), remaining(b));
        end
    end
end

function allOk = reportCheck(allOk, label, ok, detail)
% REPORTCHECK  Print one PASS/FAIL line and fold it into the running result.
    if ok, tag = 'PASS'; else, tag = 'FAIL'; end
    if isempty(detail)
        fprintf('  [%s] %s\n', tag, label);
    else
        fprintf('  [%s] %s (%s)\n', tag, label, detail);
    end
    allOk = allOk && ok;
end

function tf = segmentsCross(p1, p2, p3, p4)
% SEGMENTSCROSS  True if segment p1-p2 properly crosses segment p3-p4.
    d1 = cross2(p4 - p3, p1 - p3);   d2 = cross2(p4 - p3, p2 - p3);
    d3 = cross2(p2 - p1, p3 - p1);   d4 = cross2(p2 - p1, p4 - p1);
    tf = (d1*d2 < 0) && (d3*d4 < 0);
end

function z = cross2(u, v)
% CROSS2  z-component of the 2-D cross product.
    z = u(1)*v(2) - u(2)*v(1);
end

function [ok, assign, remaining] = packBars(len, barLengths)
% PACKBARS  First-fit-decreasing: assign each member to a stock bar.
    remaining = barLengths(:)';
    assign    = zeros(numel(len), 1);
    ok        = true;
    [~, order] = sort(len, 'descend');
    for k = order(:)'
        b = find(remaining >= len(k) - 1e-9, 1, 'first');
        if isempty(b)
            ok = false;  return;
        end
        assign(k)    = b;
        remaining(b) = remaining(b) - len(k);
    end
end

function s = formatForce(f)
% FORMATFORCE  '12.345 (T)' / '12.345 (C)' / '0.000 (0)' from a signed force.
    if abs(f) < 1e-6
        s = '0.000 (0)';
    elseif f > 0
        s = sprintf('%.3f (T)', f);
    else
        s = sprintf('%.3f (C)', abs(f));
    end
end

function ok = compareWithHand(names, Fhand, Fcode, tol)
% COMPAREWITHHAND  Print hand vs. code forces side by side and flag mismatches.
    fprintf('%-8s %-14s %-14s %-8s\n', 'Item', 'Hand', 'Code', '|diff|');
    ok = true;
    for k = 1:numel(Fhand)
        d = abs(Fhand(k) - Fcode(k));
        flag = '';
        if d > tol, flag = '  <-- MISMATCH'; ok = false; end
        fprintf('%-8s %-14s %-14s %-8.3f%s\n', names{k}, formatForce(Fhand(k)), ...
                formatForce(Fcode(k)), d, flag);
    end
end

function printTrussReport(T, res, info)
% PRINTTRUSSREPORT  Console output in the exact format of Section 2.4.4.
    fprintf('%% EK301, Section %s, %s: %s, %s.\n', info.section, info.group, ...
            info.names, info.date);
    fprintf('Load: %g oz\n', T.W);
    fprintf('Member forces in oz\n');
    for m = 1:T.M
        fprintf('m%d: %s\n', m, formatForce(res.Fmem(m)));
    end
    fprintf('Reaction forces in oz:\n');
    r = res.reactions;
    r(abs(r) < 1e-9) = 0;                 % avoid printing "-0.000"
    fprintf('Sx1: %.3f\nSy1: %.3f\nSy2: %.3f\n', r);
    fprintf('Cost of truss: $%.2f\n', res.cost);
    fprintf('Theoretical max load/cost ratio in oz/$: %.4f\n', res.ratio);
end

function printMaxLoadSummary(T, res)
% PRINTMAXLOADSUMMARY  Items 2-4 of Section 2.5.
    c = res.critical;
    fprintf('Critical member (buckles first): m%d, joints %d-%d\n', c, ...
            find(T.C(:, c))');
    fprintf('  Length                : %.3f in\n', res.len(c));
    fprintf('  Buckling strength Fb  : %.2f +/- %.2f oz\n', res.Fcrit, res.Ub(c));
    fprintf('  Ratio Rc = Fc/W       : %.4f (member is in compression)\n', res.R(c));
    fprintf('Max theoretical load    : %.2f +/- %.2f oz\n', res.Wmax, res.Wunc);
    fprintf('Total member length     : %.2f in, joints = %d\n', res.Ltot, T.J);
    fprintf('Cost                    : $%.2f\n', res.cost);
    fprintf('Load-to-cost ratio      : %.4f oz/$\n', res.ratio);
end

function printMemberTable(T, res)
% PRINTMEMBERTABLE  Results table: length, T/C, buckling strength (compression
%   members only) and force magnitude at the nominal maximum truss load.
    fprintf('%-7s %-9s %-4s %-12s %-9s %-14s\n', 'Member', 'Length(in)', 'T/C', ...
            'Fb (oz)', '+/- (oz)', 'Force@Wmax(oz)');
    for m = 1:T.M
        Fat = abs(res.R(m)) * res.Wmax;
        if abs(res.Fmem(m)) < 1e-6
            tc = '0';
        elseif res.Fmem(m) > 0
            tc = 'T';
        else
            tc = 'C';
        end
        if res.comp(m)
            fprintf('m%-6d %-10.3f %-4s %-12.2f %-9.2f %-14.2f\n', m, res.len(m), ...
                    tc, res.Fb(m), res.Ub(m), Fat);
        else
            fprintf('m%-6d %-10.3f %-4s %-12s %-9s %-14.2f\n', m, res.len(m), ...
                    tc, '--', '--', Fat);
        end
    end
end

function writeResultsCSV(T, res, filename)
% WRITERESULTSCSV  Same table as printMemberTable, saved as CSV (open in Excel).
    fid = fopen(filename, 'w');
    fprintf(fid, 'Member,Length_in,TensionCompression,Fb_oz,Ufit_oz,ForceAtWmax_oz\n');
    for m = 1:T.M
        if abs(res.Fmem(m)) < 1e-6
            tc = '0';
        elseif res.Fmem(m) > 0
            tc = 'T';
        else
            tc = 'C';
        end
        Fat = abs(res.R(m)) * res.Wmax;
        if res.comp(m)
            fprintf(fid, '%d,%.4f,%s,%.4f,%.4f,%.4f\n', m, res.len(m), tc, ...
                    res.Fb(m), res.Ub(m), Fat);
        else
            fprintf(fid, '%d,%.4f,%s,,,%.4f\n', m, res.len(m), tc, Fat);
        end
    end
    fclose(fid);
end

function plotTruss(T, res)
% PLOTTRUSS  Labeled diagram: joints, members, supports, load; critical member in red.
    figure('Name', T.name, 'Color', 'w');
    hold on;
    for m = 1:T.M
        j = find(T.C(:, m));
        xs = T.X(j);  ys = T.Y(j);
        if m == res.critical
            plot(xs, ys, '-',  'Color', [0.85 0.1 0.1], 'LineWidth', 4);
        elseif abs(res.Fmem(m)) < 1e-6
            plot(xs, ys, '--', 'Color', [0.5 0.5 0.5],  'LineWidth', 1.5);
        elseif res.Fmem(m) > 0
            plot(xs, ys, '-',  'Color', [0.0 0.35 0.85], 'LineWidth', 2);
        else
            plot(xs, ys, '-',  'Color', [0.1 0.1 0.1],  'LineWidth', 2);
        end
        text(mean(xs), mean(ys) + 0.4, sprintf('m%d', m), 'Color', [0 0.5 0], ...
             'FontWeight', 'bold', 'FontSize', 9, 'HorizontalAlignment', 'center');
    end
    plot(T.X, T.Y, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 15);
    for j = 1:T.J
        text(T.X(j), T.Y(j), sprintf('%d', j), 'FontSize', 8, ...
             'HorizontalAlignment', 'center');
    end
    % supports (drawn just below their joints) and the applied load arrow
    plot(T.X(T.pin),    T.Y(T.pin) - 1.8,    '^', 'MarkerSize', 10, ...
         'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerEdgeColor', 'k');
    plot(T.X(T.roller), T.Y(T.roller) - 1.8, 'o', 'MarkerSize', 10, ...
         'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerEdgeColor', 'k');
    lj = T.loadJoint;  xl = T.X(lj);  yl = T.Y(lj);
    plot([xl xl], [yl - 4.5, yl - 1.0], 'r-', 'LineWidth', 2);            % shaft
    plot([xl - 0.4, xl, xl + 0.4], [yl - 1.8, yl - 1.0, yl - 1.8], ...    % arrow head
         'r-', 'LineWidth', 2);
    text(xl + 0.7, yl - 3.5, sprintf('W = %g oz', T.W), 'Color', 'r');
    axis equal;  grid on;  box on;
    xlim([-3, max(T.X) + 3]);  ylim([-7, max(T.Y) + 3]);
    xlabel('x (in)');  ylabel('y (in)');
    title(sprintf('%s  -  m%d buckles first at %.1f oz', T.name, res.critical, res.Wmax));
    text(-2, -6.3, ['blue = tension, black = compression, dashed = zero-force, ' ...
                   'red = first to buckle'], 'FontSize', 8);
end

function saveDesignFile(T, filename)
% SAVEDESIGNFILE  Save the six required variables to a .mat input file.
    C = T.C;  Sx = T.Sx;  Sy = T.Sy;  X = T.X;  Y = T.Y;  L = T.L; %#ok<NASGU>
    save(filename, 'C', 'Sx', 'Sy', 'X', 'Y', 'L');
end